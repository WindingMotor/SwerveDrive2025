// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.generic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SUB_RotationController;
import frc.robot.util.ExpDecayFF.RotationState;

public class CMD_RotationController extends Command {
	private final RotationState rotationState;
	private SUB_RotationController rotationControllerHusk;

	public CMD_RotationController(
			SUB_RotationController rotationControllerHusk, RotationState rotationState) {
		this.rotationControllerHusk = rotationControllerHusk;
		this.rotationState = rotationState;
		addRequirements(rotationControllerHusk);
	}

	@Override
	public void initialize() {
		rotationControllerHusk.updateRotationState(rotationState);
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
