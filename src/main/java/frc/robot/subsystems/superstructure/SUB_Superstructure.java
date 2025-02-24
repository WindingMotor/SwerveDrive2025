// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.led.SUB_Led;
import frc.robot.subsystems.superstructure.SuperstructureState.State;
import org.littletonrobotics.junction.Logger;

public class SUB_Superstructure extends SubsystemBase {
	private SuperstructureState.State currentSuperstructureState = SuperstructureState.IDLE;

	public State currentDynamicEjectState =
			SuperstructureState.createState("EJECT_DYNAMIC", 0.5, 135, 18);

	public SUB_Intake intake;
	public SUB_Elevator elevator;
	public SUB_Led led;
	public Drive drive;

	private boolean previousIntakeSensorState = false;

	public SUB_Superstructure(Drive drive, SUB_Intake intake, SUB_Elevator elevator, SUB_Led led) {
		this.drive = drive;
		this.intake = intake;
		this.elevator = elevator;
		this.led = led;
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

		// Check for sensor state change from false to true
		if (!previousIntakeSensorState
				&& intake.getSensorState()
				&& DriverStation.isEnabled()
				&& currentSuperstructureState == SuperstructureState.CORAL_STATION) {
			CommandScheduler.getInstance()
					.schedule(new CMD_Superstructure(this, SuperstructureState.IDLE));
		}

		int closestTagId = drive.getRecentClosestTagData().getFirst();
		double distanceM = drive.getRecentClosestTagData().getSecond();

		double minDist = MIN_DIST_TELEOP;

		if (DriverStation.isAutonomousEnabled()) {
			// minDist = MIN_DIST_AUTO;
			// Do nothing in auto
		} else {

			if (closestTagId == -1 || distanceM >= minDist) {

				return;
			} else {
				switch (closestTagId) {
					case 2:
					case 1:
					case 12:
					case 13:
						CommandScheduler.getInstance()
								.schedule(new CMD_Superstructure(this, SuperstructureState.CORAL_STATION));
						break;

					case 7:
						break;

					case 8:
						break;

					case 6:
						break;

					case 9:
						break;

					case 10:
						break;

					case 11:
						break;

					default:
						break;
				}
			}
		}

		previousIntakeSensorState = intake.getSensorState();
	}
}
