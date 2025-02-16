// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.generic.CMD_RotationController;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.superstructure.SUB_Superstructure;
import frc.robot.superstructure.SuperstructureState;
import frc.robot.swerve.SUB_Swerve;
import frc.robot.util.ExpDecayFF.RotationState;

public class CommandRegistrar {

	public static void registerCommands(SUB_Swerve swerve, SUB_Superstructure superstructure) {

		NamedCommands.registerCommand("Rot_Source", new CMD_RotationController(RotationState.BACKWARD));
		NamedCommands.registerCommand(
				"Intake_Coral", new CMD_Superstructure(superstructure, SuperstructureState.CORAL_STATION));
	}
}
