// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CMD_DriveAlign;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.InputConstants;
import frc.robot.intake.IO_IntakeReal;
import frc.robot.intake.SUB_Intake;
import frc.robot.swerve.IO_SwerveReal;
import frc.robot.swerve.SUB_Swerve;
import frc.robot.vision.IO_VisionReal;
import frc.robot.vision.IO_VisionSim;
import frc.robot.vision.SUB_Vision;
import frc.robot.webserver.WebServer;
import java.io.File;

public class RobotContainer {
	private final CommandXboxController driverController;

	private final WebServer webServer;

	private final SUB_Swerve swerve;
	private final SUB_Vision vision;
	private final SUB_Intake intake;

	private final InputConstants globalInputMap;

	public RobotContainer() {

		webServer = new WebServer();

		driverController = new CommandXboxController(0); // port 0
		globalInputMap = InputConstants.TX16S_MAIN; // Set the global input map to Xbox Controller

		vision = new SUB_Vision(Robot.isSimulation() ? new IO_VisionSim() : new IO_VisionReal());

		swerve =
				new SUB_Swerve(
						vision, new IO_SwerveReal(new File(Filesystem.getDeployDirectory(), "swerve")));

		intake = new SUB_Intake(new IO_IntakeReal());

		configureDefaultCommands();
		configureWebserverCommands();
		configurePathPlannerCommands();
		configureButtonBindings();
	}

	private void configureDefaultCommands() {

		swerve.setDefaultCommand(
				new CMD_DriveAlign(swerve, driverController, globalInputMap)); // Drive and aim at speaker
	}

	private void configureWebserverCommands() {
		webServer.registerCommand("T1", swerve.driveToPose(FieldConstants.BLUE_TOP_TOP_LEFT));
		webServer.registerCommand("T2", swerve.driveToPose(FieldConstants.BLUE_TOP_TOP_RIGHT));

		webServer.registerCommand("TL1", swerve.driveToPose(FieldConstants.BLUE_TOP_LEFT_BOTTOM));
		webServer.registerCommand("TL2", swerve.driveToPose(FieldConstants.BLUE_TOP_LEFT_TOP));
	}

	private void configurePathPlannerCommands() {

		NamedCommands.registerCommand("Intake_Algae", new PrintCommand("Hi"));
	}

	private void configureButtonBindings() {

		// Aim at 0,0 for testing
		// driverController.a().onTrue(new CMD_AimAtPose(swerve, new Pose2d(), 0.1));

		// Aim at tag 16 for testing
		// driverController.b().onTrue(new CMD_AimAtAprilTag(swerve, 16, 0.1));

		driverController.a().onTrue(intake.setState(SUB_Intake.State.ALGAE_GROUND));
		driverController.a().toggleOnFalse(intake.setState(SUB_Intake.State.STOWED));
	}

	public Command getAutonomousCommand() {
		return swerve.getAutonomousCommand("Test");
	}
}

//Ben was here