// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.elevator.SUB_Elevator;
import frc.robot.intake.SUB_Intake;
import frc.robot.superstructure.SuperstructureState.State;
import frc.robot.util.SUB_Led;
import org.littletonrobotics.junction.Logger;

public class SUB_Superstructure extends SubsystemBase {
	private SuperstructureState.State currentSuperstructureState = SuperstructureState.IDLE;
	private SuperstructureState.State previousSuperstructureState = SuperstructureState.IDLE;

	public State currentDynamicEjectState =
			SuperstructureState.createState("EJECT_DYNAMIC", 0.5, 135, 18);

	public SUB_Intake intake;
	public SUB_Elevator elevator;
	public SUB_Led led;

	private boolean previousIntakeSensorState = false;

	public SUB_Superstructure(SUB_Intake intake, SUB_Elevator elevator, SUB_Led led) {
		this.intake = intake;
		this.elevator = elevator;
		this.led = led;
	}

	public void updateSuperstructureState(SuperstructureState.State newSuperstructureState) {
		previousSuperstructureState = currentSuperstructureState;
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
