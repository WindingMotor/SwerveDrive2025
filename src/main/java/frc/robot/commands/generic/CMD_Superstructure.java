// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.generic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.SUB_Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class CMD_Superstructure extends Command {
	private final SUB_Superstructure superstructure;
	private final SuperstructureState.State newSuperstructureState;

	public CMD_Superstructure(
			SUB_Superstructure superstructure, SuperstructureState.State newSuperstructureState) {
		this.superstructure = superstructure;
		this.newSuperstructureState = newSuperstructureState;
		addRequirements(superstructure);
	}

	@Override
	public void initialize() {
		superstructure.updateSuperstructureState(newSuperstructureState);
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
