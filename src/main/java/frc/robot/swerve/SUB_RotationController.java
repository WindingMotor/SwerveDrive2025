// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DynamicConstants;
import frc.robot.util.ExpDecayFF.RotationState;

public class SUB_RotationController extends SubsystemBase {

	public SUB_RotationController() {}

	@Override
	public void periodic() {}

	public void updateRotationState(RotationState newRotationState) {
		DynamicConstants.GLOBAL_ROTATION_STATE = newRotationState;
	}
}
