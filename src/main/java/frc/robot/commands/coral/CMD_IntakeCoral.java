// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.subsystems.superstructure.SUB_Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class CMD_IntakeCoral extends SequentialCommandGroup {
	public CMD_IntakeCoral(SUB_Superstructure superstructure) {

		addCommands(
				new CMD_Superstructure(superstructure, SuperstructureState.CORAL_STATION),
				new CMD_Superstructure(
						superstructure,
						superstructure.setAndGetEjectState(SuperstructureState.CORAL_STATION.getSpeed())));
	}
}
