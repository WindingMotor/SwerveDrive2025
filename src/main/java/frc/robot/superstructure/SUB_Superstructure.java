// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.elevator.SUB_Elevator;
import frc.robot.intake.SUB_Intake;
import frc.robot.superstructure.SuperstructureState.State;
import frc.robot.swerve.SUB_Swerve;
import frc.robot.util.SUB_Led;
import org.littletonrobotics.junction.Logger;

public class SUB_Superstructure extends SubsystemBase {
	private SuperstructureState.State currentSuperstructureState = SuperstructureState.IDLE;

	public State currentDynamicEjectState =
			SuperstructureState.createState("EJECT_DYNAMIC", 0.5, 135, 18);

	public SUB_Intake intake;
	public SUB_Elevator elevator;
	public SUB_Led led;
	private SUB_Swerve swerve;

	private boolean previousIntakeSensorState = false;

	public SUB_Superstructure(
			SUB_Intake intake, SUB_Elevator elevator, SUB_Led led, SUB_Swerve swerve) {
		this.intake = intake;
		this.elevator = elevator;
		this.led = led;
		this.swerve = swerve;
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

	double minDistTeleop = 1.2;
	double minDistAuto = 2.5;

	@Override
	public void periodic() {

		/*
		// Check for sensor state change from false to true
		if (!previousIntakeSensorState
				&& intake.getSensorState()
				&& DriverStation.isEnabled()
				&& currentSuperstructureState == SuperstructureState.CORAL_STATION) {
			CommandScheduler.getInstance()
					.schedule(new CMD_Superstructure(this, SuperstructureState.IDLE));
		}

		Pair<Integer, Double> closestTagData = swerve.getClosestAprilTagID();
		int closestTagId = closestTagData.getFirst();
		double closestTagDistanceM = closestTagData.getSecond();

		double minDist = minDistTeleop;
		if (DriverStation.isAutonomousEnabled()) {
			minDist = minDistAuto;
		}

		if (closestTagId == -1 || closestTagDistanceM >= minDist) {
			DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.NONE;
			return;
		} else {
			switch (closestTagId) {
				case 2:
					DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.SOURCE_RIGHT;
					CommandScheduler.getInstance()
							.schedule(new CMD_Superstructure(this, SuperstructureState.CORAL_STATION));
					break;

				case 7:
					DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.REEF_BOTTOM;
					break;

				case 8:
					DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.REEF_BOTTOM_RIGHT;
					break;

				case 6:
					DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.REEF_BOTTOM_LEFT;
					break;

				case 9:
					DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.REEF_TOP_RIGHT;
					break;

				case 10:
					DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.REEF_TOP;
					break;

				case 11:
					DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.REEF_TOP_LEFT;
					break;

				default:
					DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.NONE;
					break;
			}
		}
			*/

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
	}
}
