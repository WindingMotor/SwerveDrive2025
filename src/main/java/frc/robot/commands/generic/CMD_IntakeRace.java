// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.generic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class CMD_IntakeRace extends Command {
	private final SUB_Intake intake;
	private boolean isDone = false;

	public CMD_IntakeRace(SUB_Intake intake) {
		this.intake = intake;
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		isDone = false;
		intake.updateLocalState(SuperstructureState.CORAL_STATION);
	}

	@Override
	public void execute() {
		if (intake.getSensorState()) {
			isDone = true;
		}
	}

	@Override
	public boolean isFinished() {
		return isDone;
	}

	@Override
	public void end(boolean interrupted) {}
}
