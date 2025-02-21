// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.subsystems.superstructure.SUB_Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class CMD_IntakeAlgae extends SequentialCommandGroup {
	public CMD_IntakeAlgae(SUB_Superstructure superstructure) {
		addCommands(
				// Move to Algae Ground state
				new CMD_Superstructure(superstructure, SuperstructureState.ALGAE_GROUND)

				// TODO: Auto idle intake after algae is NOT sensed by the sensor and current limit is
				// reached
				// new CMD_Superstructure(superstructure, SuperstructureState.State.IDLE)
				);
	}
}
