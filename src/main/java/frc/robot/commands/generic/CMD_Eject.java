// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.generic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.SUB_Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class CMD_Eject extends Command {
	private final SUB_Superstructure superstructure;

	public CMD_Eject(SUB_Superstructure superstructure) {
		this.superstructure = superstructure;
		addRequirements(superstructure);
	}

	@Override
	public void initialize() {
		SuperstructureState.State newState = superstructure.setAndGetEjectState(-0.6);
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
