// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.superstructure.SuperstructureState;
import org.littletonrobotics.junction.Logger;

public class SUB_Intake extends SubsystemBase {
	private final IO_IntakeBase io;
	private final IO_IntakeBase.IntakeInputs inputs = new IO_IntakeBase.IntakeInputs();
	private SuperstructureState.State localState = SuperstructureState.IDLE;

	public SUB_Intake(IO_IntakeBase io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		io.setArmAngle(localState.getDeg());
		io.setIntakeSpeed(localState.getSpeed());
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);
	}

	public void updateLocalState(SuperstructureState.State newLocalState) {
		localState = newLocalState;
	}

	public SuperstructureState.State getCurrentLocalState() {
		return localState;
	}
}
