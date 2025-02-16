// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.generic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DynamicConstants;
import frc.robot.util.ExpDecayFF.RotationState;

public class CMD_RotationController extends Command {
	private final RotationState rotationState;

	public CMD_RotationController(RotationState rotationState) {
		this.rotationState = rotationState;
		addRequirements();
	}

	@Override
	public void initialize() {
		DynamicConstants.GLOBAL_ROTATION_STATE = rotationState;
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
