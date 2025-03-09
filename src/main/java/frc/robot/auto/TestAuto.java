// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.SUB_Superstructure;

public class TestAuto extends SequentialCommandGroup {

	public TestAuto(Drive drive, SUB_Superstructure superstructure) {
		/*
			try {

				addCommands(
						new PrintCommand("Starting auton"),
						AutoBuilder.followPath(PathPlannerPath.fromPathFile("T1")),
						new PrintCommand("Starting wait"),
						new WaitCommand(12),
						new PrintCommand("Starting align"),
						DriveCommands.driveToZone(drive, ZonePose.REEF_TOP_RIGHT_BOTTOM),
						new PrintCommand("End"));

			} catch (FileVersionException | IOException | ParseException e) {

				e.printStackTrace();
			}
		}
			*/
	}
}
