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
import frc.robot.util.SUB_Led;
import org.littletonrobotics.junction.Logger;

public class SUB_Superstructure extends SubsystemBase {
	private SuperstructureState.State currentSuperstructureState = SuperstructureState.IDLE;
	public SUB_Intake intake;
	public SUB_Elevator elevator;
	public SUB_Led led;

	public SUB_Superstructure(SUB_Intake intake, SUB_Elevator elevator, SUB_Led led) {
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

	public State getCurrentStateWithNewWheelSpeed(double newSpeed) {
		return SuperstructureState.createState(
				currentSuperstructureState.getName(),
				currentSuperstructureState.getHeightM(),
				currentSuperstructureState.getDeg(),
				newSpeed);
	}

	public SuperstructureState.State getCurrentSuperstructureState() {
		return currentSuperstructureState;
	}
}
