// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.InputConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.swerve.SUB_Swerve;
import java.util.List;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class CMD_Drive extends Command {

	private final SUB_Swerve swerve;
	private final CommandXboxController controller;
	private final InputConstants controllerMap;
	private boolean resetHeading = false;

	public CMD_Drive(
			SUB_Swerve swerve, CommandXboxController controller, InputConstants controllerMap) {

		this.swerve = swerve;
		this.controller = controller;
		this.controllerMap = controllerMap;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		resetHeading = true;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double headingX = 0;
		double headingY = 0;

		double vX = controller.getRawAxis(controllerMap.forwardAxis);
		double vY = controller.getRawAxis(controllerMap.strafeAxis);
		double headingAdjust = controller.getRawAxis(controllerMap.rotationAxis);

		// Prevent Movement After Auto
		if (resetHeading) {
			if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust) == 0) {
				// Get the curret Heading
				Rotation2d currentHeading = swerve.getHeading();

				// Set the Current Heading to the desired Heading
				headingX = currentHeading.getSin();
				headingY = currentHeading.getCos();
			}
			// Dont reset Heading Again
			resetHeading = false;
		}

		// Scale inputs
		Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(vX, vY));

		// Get desired speeds
		ChassisSpeeds desiredSpeeds =
				swerve
						.getSwerveController()
						.getTargetSpeeds(
								scaledInputs.getX(),
								scaledInputs.getY(),
								headingX,
								headingY,
								swerve.getHeading().getRadians(),
								RobotConstants.MAX_SPEED);

		// Limit velocity to prevent tippy
		Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
		translation =
				SwerveMath.limitVelocity(
						translation,
						swerve.getFieldVelocity(),
						swerve.getPose(),
						RobotConstants.LOOP_TIME,
						RobotConstants.ROBOT_MASS,
						List.of(RobotConstants.CHASSIS),
						swerve.getSwerveDriveConfiguration());
		SmartDashboard.putNumber("LimitedTranslation", translation.getX());
		SmartDashboard.putString("Translation", translation.toString());

		// Make the robot move
		if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust) > 0) {
			resetHeading = true;
			swerve.drive(translation, (6.00 * -headingAdjust), true);
		} else {
			swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		// Stop the robot when the command ends
		swerve.drive(new Translation2d(), 0, true);
		// Disable motor brake mode
		swerve.setMotorBrake(false);
	}
}
