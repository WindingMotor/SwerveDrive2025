// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.InputConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.swerve.SUB_Swerve;
import frc.robot.util.ExpDecayFF.RotationState;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class CMD_Drive extends Command {
	private final SUB_Swerve swerve;
	private final CommandXboxController controller;
	private final InputConstants controllerMap;
	private final Translation2d zeroTranslation = new Translation2d();
	private ChassisSpeeds desiredSpeeds;
	private Translation2d translation;
	SwerveController swerveController;

	public CMD_Drive(
			SUB_Swerve swerve, CommandXboxController controller, InputConstants controllerMap) {
		this.swerve = swerve;
		this.controller = controller;
		this.controllerMap = controllerMap;
		swerveController = swerve.getSwerveController();
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		swerve.setMotorBrake(true);
	}

	@Override
	public void execute() {
		// Get controller inputs once
		double vX = -controller.getRawAxis(controllerMap.forwardAxis);
		double vY = controller.getRawAxis(controllerMap.strafeAxis);
		double headingAdjust = -controller.getRawAxis(controllerMap.rotationAxis);
		double currentHeading = swerve.getHeading().getRadians();
		double currentYaw = -swerve.inputs.gyroYawDegrees;

		// Update rotation state
		boolean isButton3Pressed = controller.button(3).getAsBoolean();
		if (isButton3Pressed) {
			// swerve.getRotationFFController().setState(RotationState.LEFT);
			DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.LEFT;
		} else {
			// DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.NONE;
		}

		/*
			Pair<Integer, Double> closestTagData = swerve.getClosestAprilTagID();
			int closestTagId = closestTagData.getFirst();
			double closestTagDistanceM = closestTagData.getSecond();

			if (closestTagId != -1 && closestTagDistanceM < 0.5) {
				if (closestTagId == 7) {
					DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.FORWARD;
				} else if (closestTagId == 6) {
					DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.BACKWARD;
				}
			}

		} else if (swerve.getRotationFFController().getState() != RotationState.NONE) {
			DynamicConstants.GLOBAL_ROTATION_STATE = RotationState.NONE;
		}

		*/
		// Scale inputs once
		Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(vX, vY));
		double scaledX = scaledInputs.getX();
		double scaledY = scaledInputs.getY();

		// Calculate speeds
		desiredSpeeds =
				swerveController.getTargetSpeeds(
						scaledX, scaledY, 0, 0, currentHeading, RobotConstants.MAX_SPEED);

		desiredSpeeds.omegaRadiansPerSecond = -headingAdjust * 6.0;
		translation = SwerveController.getTranslation2d(desiredSpeeds);

		// Drive and log
		swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
		logData(translation, currentYaw);
	}

	private void logData(Translation2d translation, double currentYaw) {
		Logger.recordOutput("Drive/Translation", translation.toString());
		Logger.recordOutput("Drive/Target Angle", swerve.getRotationFFController().getTargetAngle());
		Logger.recordOutput("Drive/Current Angle", currentYaw);
		Logger.recordOutput("Drive/Rot State", swerve.getRotationFFController().getState().toString());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		swerve.drive(zeroTranslation, 0, true);
		swerve.setMotorBrake(false);
		swerve.getRotationFFController().setState(RotationState.NONE);
	}
}
