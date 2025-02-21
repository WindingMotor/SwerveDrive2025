// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.generic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class CMD_Elevator extends Command {
	private final SUB_Elevator elevator;
	// private final SUB_Led led;
	private final SuperstructureState.State state;

	public CMD_Elevator(SUB_Elevator elevator, SuperstructureState.State state) {
		this.elevator = elevator;
		//	this.led = led;
		this.state = state;
		addRequirements(elevator);
	}

	@Override
	public void initialize() {
		elevator.updateLocalState(state);
		//	led.updateLocalState(state);
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
