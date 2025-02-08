// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.generic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.SUB_Superstructure;
import frc.robot.superstructure.SuperstructureState;

public class CMD_Idle extends Command {
	private final SUB_Superstructure superstructure;

	public CMD_Idle(SUB_Superstructure superstructure) {
		this.superstructure = superstructure;
		addRequirements(superstructure);
	}

	@Override
	public void initialize() {
		// Set to idle
		superstructure.updateSuperstructureState(SuperstructureState.IDLE);
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
