// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

public class CommandRegistrar {

	/*
	public static void registerCommands(SUB_Swerve swerve, SUB_Superstructure superstructure) {


		SUB_RotationController rotationControllerHusk = new SUB_RotationController();

		NamedCommands.registerCommand(
				"Source_Right",
				new CMD_RotationController(rotationControllerHusk, RotationState.SOURCE_RIGHT));

		NamedCommands.registerCommand(
				"Reef_Top_Right",
				new CMD_RotationController(rotationControllerHusk, RotationState.REEF_TOP_RIGHT));

		new ParallelCommandGroup(
				new CMD_RotationController(
						superstructure.rotationControllerHusk, RotationState.SOURCE_RIGHT),
				new CMD_Superstructure(superstructure, SuperstructureState.CORAL_STATION)));



		NamedCommands.registerCommand("Source_Left", new PrintCommand("Hello"));

				NamedCommands.registerCommand(
						"Source_Left",
						new ParallelCommandGroup(
								new CMD_RotationController(superstructure.rotationControllerHusk, RotationState.LEFT),
							new CMD_Superstructure(superstructure, SuperstructureState.CORAL_STATION)));


		NamedCommands.registerCommand(
				"Intake_Coral", new CMD_Superstructure(superstructure, SuperstructureState.CORAL_STATION));

		NamedCommands.registerCommand(
				"L4", new CMD_Superstructure(superstructure, SuperstructureState.L4_SCORING));

		NamedCommands.registerCommand("Eject", new CMD_Eject(superstructure));
	}
	*/
}
