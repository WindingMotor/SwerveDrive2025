// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveCommands.ZonePose;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.led.SUB_Led;
import frc.robot.subsystems.superstructure.SuperstructureState.State;
import org.littletonrobotics.junction.Logger;

public class SUB_Superstructure extends SubsystemBase {

	public static ZonePose globalFirstPose = ZonePose.NONE;
	public static ZonePose globalSecondPose = ZonePose.NONE;

	private SuperstructureState.State currentSuperstructureState = SuperstructureState.IDLE;

	public State currentDynamicEjectState =
			SuperstructureState.createState("EJECT_DYNAMIC", 0.5, 135, .5);

	public State currentDynamicAlage = SuperstructureState.IDLE;

	private Pair<ZonePose, ZonePose> localAutoAlignZone = Pair.of(ZonePose.NONE, ZonePose.NONE);

	public SUB_Intake intake;
	public SUB_Elevator elevator;
	public SUB_Led led;
	public Drive drive;

	private boolean previousIntakeSensorState = false;

	private CommandXboxController operatorController;

	public SUB_Superstructure(
			Drive drive,
			SUB_Intake intake,
			SUB_Elevator elevator,
			SUB_Led led,
			CommandXboxController operatorController) {
		this.drive = drive;
		this.intake = intake;
		this.elevator = elevator;
		this.led = led;
		this.operatorController = operatorController;
	}

	public void updateSuperstructureState(SuperstructureState.State newSuperstructureState) {
		currentSuperstructureState = newSuperstructureState;

		elevator.updateLocalState(currentSuperstructureState);
		intake.updateLocalState(currentSuperstructureState);
		led.updateLocalState(currentSuperstructureState);

		Logger.recordOutput("Superstructure/State", currentSuperstructureState.toString());
		Logger.recordOutput("Superstructure/Name", currentSuperstructureState.getName());
		Logger.recordOutput("Superstructure/HeightM", currentSuperstructureState.getHeightM());
		Logger.recordOutput("Superstructure/Deg", currentSuperstructureState.getDeg());
		Logger.recordOutput("Superstructure/Speed", currentSuperstructureState.getSpeed());
	}

	public State setAndGetEjectState(double newWheelSpeed) {
		currentDynamicEjectState =
				SuperstructureState.createState(
						"EJECT_DYNAMIC",
						currentSuperstructureState.getHeightM(),
						currentSuperstructureState.getDeg(),
						newWheelSpeed);
		return currentDynamicEjectState;
	}

	public SuperstructureState.State getCurrentSuperstructureState() {
		return currentSuperstructureState;
	}

	double MIN_DIST_TELEOP = 1.5;
	double MIN_DIST_AUTO = 2;

	@Override
	public void periodic() {

		// Check for sensor state changes and update current limit accordingly
		if (intake.getSensorState() != previousIntakeSensorState
				&& (currentSuperstructureState == SuperstructureState.CORAL_STATION)) {
			// If sensor changed to true, do the no spin idle
			if (intake.getSensorState()) {
				updateSuperstructureState(SuperstructureState.IDLE_CALM);
			}
		}

		// Update operator controller rumble
		double matchTimeRemaining = DriverStation.getMatchTime();
		boolean isEndgame = matchTimeRemaining <= 15.0 && matchTimeRemaining > 0;

		if (isEndgame) {
			operatorController.setRumble(RumbleType.kBothRumble, 0.4);
		} else {
			operatorController.setRumble(RumbleType.kBothRumble, 0.0);
		}

		int closestTagId = drive.getRecentClosestTagData().getFirst();
		double distanceM = drive.getRecentClosestTagData().getSecond();

		double minDist = MIN_DIST_TELEOP;

		if (DriverStation.isAutonomousEnabled()) {
			// Do nothing in auto
		} else {

			Logger.recordOutput("Superstructure/DynamicAlage", currentDynamicAlage.getName());

			if (closestTagId == -1 || distanceM >= minDist) {

				return;
			} else {
				switch (closestTagId) {

						// Source
					case 2:
					case 1:
					case 12:
					case 13:
						currentDynamicAlage = SuperstructureState.ALGAE_GROUND;
						if (!intake.getSensorState()
								&& currentSuperstructureState != SuperstructureState.ALGAE_GROUND) {
							CommandScheduler.getInstance()
									.schedule(new CMD_Superstructure(this, SuperstructureState.CORAL_STATION));
						}
						break;

						// Bottom Face
					case 18:
					case 7:
						currentDynamicAlage = SuperstructureState.ALGAE_L3;
						localAutoAlignZone = getBottomPose();
						break;

						// Bottom Right Face
					case 17:
					case 8:
						currentDynamicAlage = SuperstructureState.ALGAE_L2;
						localAutoAlignZone = getBottomRight();
						break;

						// Bottom Left Face
					case 19:
					case 6:
						currentDynamicAlage = SuperstructureState.ALGAE_L2;
						localAutoAlignZone = getBottomLeft();
						break;

						// Top Right Face
					case 22:
					case 9:
						currentDynamicAlage = SuperstructureState.ALGAE_L3;
						localAutoAlignZone = getTopRightPose();
						break;

						// Top Face
					case 21:
					case 10:
						currentDynamicAlage = SuperstructureState.ALGAE_L2;
						localAutoAlignZone = getTopPose();
						break;

						// Top Left Face
					case 20:
					case 11:
						currentDynamicAlage = SuperstructureState.ALGAE_L3;
						localAutoAlignZone = getTopLeftPose();
						break;

						// Processor
					case 3:
					case 16:
						currentDynamicAlage = SuperstructureState.ALGAE_PROCESSOR;
						break;

					default:
						break;
				}
			}
		}

		/*

		if (previousIntakeSensorState
				&& !intake.getSensorState()
				&& DriverStation.isEnabled()
				&& (previousSuperstructureState == SuperstructureState.L1_SCORING
						|| previousSuperstructureState == SuperstructureState.L2_SCORING
						|| previousSuperstructureState == SuperstructureState.L3_SCORING
						|| previousSuperstructureState == SuperstructureState.L4_SCORING)
				&& currentSuperstructureState == currentDynamicEjectState) {
			CommandScheduler.getInstance()
					.schedule(new CMD_Superstructure(this, SuperstructureState.CORAL_STATION));
		}
					*/

		previousIntakeSensorState = intake.getSensorState();

		globalFirstPose = localAutoAlignZone.getFirst();
		globalSecondPose = localAutoAlignZone.getSecond();

		Logger.recordOutput("AutoAlign/GlobalFirst", globalFirstPose);
		Logger.recordOutput("AutoAlign/GlobalFirstPOSE", globalFirstPose.getPose());

		Logger.recordOutput("AutoAlign/GlobalSecond", globalSecondPose);
		Logger.recordOutput("AutoAlign/GlobalSecondPOSE", globalSecondPose.getPose());
	}

	public State getCurrentDynamicAlage() {
		return currentDynamicAlage;
	}

	private Pair<ZonePose, ZonePose> getTopPose() {
		return Pair.of(ZonePose.REEF_TOP_LEFT, ZonePose.REEF_TOP_RIGHT);
	}

	private Pair<ZonePose, ZonePose> getTopRightPose() {
		return Pair.of(ZonePose.REEF_TOP_RIGHT_BOTTOM, ZonePose.REEF_TOP_RIGHT_TOP);
	}

	private Pair<ZonePose, ZonePose> getTopLeftPose() {
		return Pair.of(ZonePose.REEF_TOP_LEFT_BOTTOM, ZonePose.REEF_TOP_LEFT_TOP);
	}

	private Pair<ZonePose, ZonePose> getBottomPose() {
		return Pair.of(ZonePose.REEF_BOTTOM_LEFT, ZonePose.REEF_BOTTOM_RIGHT);
	}

	private Pair<ZonePose, ZonePose> getBottomRight() {
		return Pair.of(ZonePose.REEF_BOTTOM_RIGHT_BOTTOM, ZonePose.REEF_BOTTOM_RIGHT_TOP);
	}

	private Pair<ZonePose, ZonePose> getBottomLeft() {
		return Pair.of(ZonePose.REEF_BOTTOM_LEFT_BOTTOM, ZonePose.REEF_BOTTOM_LEFT_TOP);
	}

	public Command dynamicAlage() {
		return new InstantCommand(() -> updateSuperstructureState(currentDynamicAlage), this);
	}
}
