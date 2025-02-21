// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.SUB_Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class CMD_ElevatorAlgae extends Command {
	private final SUB_Superstructure superstructure;
	private final boolean isMovingUp;

	public CMD_ElevatorAlgae(SUB_Superstructure superstructure, boolean isMovingUp) {
		this.superstructure = superstructure;
		this.isMovingUp = isMovingUp;
		addRequirements(superstructure);
	}

	@Override
	public void initialize() {
		SuperstructureState.State currentState = superstructure.getCurrentSuperstructureState();
		SuperstructureState.State newState = SuperstructureState.ALGAE_PROCESSOR;

		if (isMovingUp) {
			if (currentState == SuperstructureState.ALGAE_PROCESSOR) {
				newState = SuperstructureState.ALGAE_L2;
			} else if (currentState == SuperstructureState.ALGAE_L2) {
				newState = SuperstructureState.ALGAE_L3;
			} else if (currentState == SuperstructureState.ALGAE_L3) {
				newState = SuperstructureState.ALGAE_BARGE;
			} else if (currentState == SuperstructureState.ALGAE_BARGE) {
				newState = SuperstructureState.ALGAE_PROCESSOR;
			}
		} else {
			if (currentState == SuperstructureState.ALGAE_BARGE) {
				newState = SuperstructureState.ALGAE_L3;
			} else if (currentState == SuperstructureState.ALGAE_L3) {
				newState = SuperstructureState.ALGAE_L2;
			} else if (currentState == SuperstructureState.ALGAE_L2) {
				newState = SuperstructureState.ALGAE_PROCESSOR;
			} else if (currentState == SuperstructureState.ALGAE_PROCESSOR) {
				newState = SuperstructureState.ALGAE_BARGE;
			}
		}
		superstructure.updateSuperstructureState(newState);
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {}
}
