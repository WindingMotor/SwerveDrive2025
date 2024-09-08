// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoSelector;
import frc.robot.subsystems.swerve.IO_SwerveReal;
import frc.robot.subsystems.swerve.SUB_Swerve;

public class RobotContainer {

	// The driver's controller
	private final CommandXboxController driverController = new CommandXboxController(1);

	// The robot's subsystems and commands are defined here...
	private final IO_SwerveReal swerveIO = new IO_SwerveReal();
	private final SUB_Swerve swerveSubsystem = new SUB_Swerve(swerveIO, driverController);

	private final AutoSelector autoSelector = new AutoSelector();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the default command for the swerve subsystem
		swerveSubsystem.setDefaultCommand(
				swerveSubsystem.drive(
						() -> -driverController.getRawAxis(1),
						() -> -driverController.getRawAxis(0),
						() -> -driverController.getRawAxis(3)));

		configureBindings();
	}

	private void configureBindings() {}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// Define the autonomous command here
		return autoSelector.getSelectedAuto();
	}
}
