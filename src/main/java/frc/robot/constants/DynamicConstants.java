// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import frc.robot.util.math.ExpDecayFF.RotationState;

public final class DynamicConstants {

	public static RotationState GLOBAL_ROTATION_STATE = RotationState.NONE;

	public static void updateRotationState(RotationState newRotationState) {
		GLOBAL_ROTATION_STATE = newRotationState;
	}
}
