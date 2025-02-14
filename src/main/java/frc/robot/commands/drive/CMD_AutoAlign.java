// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.swerve.SUB_Swerve;
import frc.robot.util.ExpDecayFF;
import frc.robot.util.ExpDecayFF.RotationState;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveController;

public class CMD_AutoAlign extends Command {
	private final SUB_Swerve swerve;
	private RotationState rotationState = RotationState.NONE;
	private boolean isFinished = false;

	private ChassisSpeeds desiredSpeeds;
	private Translation2d translation;

	ExpDecayFF swerveRotationController;
	SwerveController swerveController;

	PIDController xPID = new PIDController(0.05, 0, 0);
	PIDController yPID = new PIDController(0.05, 0, 0);

	private final double positionToleranceInches = 1.0; // Inches
	private final double positionKS = 0.02;
	private final double positionIZone = 4.0;

	public CMD_AutoAlign(
			SUB_Swerve swerve, Translation2d targetTranslation, RotationState rotationState) {
		this.swerve = swerve;
		swerveController = swerve.getSwerveController();
		swerveRotationController = swerve.getRotationFFController();
		addRequirements(swerve);

		xPID.setIZone(positionIZone); // Only use Integral term within this range
		xPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
		xPID.setTolerance(positionToleranceInches);

		yPID.setIZone(positionIZone); // Only use Integral term within this range
		yPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
		yPID.setTolerance(positionToleranceInches);

		// Set the target (PID is in inches so convert)
		xPID.setSetpoint(Units.metersToInches(targetTranslation.getX()));
		yPID.setSetpoint(Units.metersToInches(targetTranslation.getY()));
	}

	@Override
	public void initialize() {
		swerve.setMotorBrake(true);
		xPID.reset();
		yPID.reset();
	}

	@Override
	public void execute() {

		Pose2d currentPose = swerve.getPose();
		double currentHeading = swerve.getHeading().getRadians();
		double currentYaw = -swerve.inputs.gyroYawDegrees;

		// Check if we are close enough to end command
		if (swerveRotationController.atTarget(currentHeading)) {
			isFinished = true;
		}

		// Calculate xPID values
		double xCorrection = xPID.calculate(Units.metersToInches(currentPose.getX()));
		double xFeedForward = positionKS * Math.signum(xCorrection);
		double vX = MathUtil.clamp(xCorrection + xFeedForward, -1.0, 1.0);

		// Calculate yPID values
		double yCorrection = yPID.calculate(Units.metersToInches(currentPose.getY()));

		double yFeedForward = positionKS * Math.signum(yCorrection);
		double vY = MathUtil.clamp(yCorrection + yFeedForward, -1.0, 1.0);

		// Set rotation state
		swerveRotationController.setState(rotationState);

		// Calculate speeds (we dont need to update the omegaRadians as rotation state handles it)
		desiredSpeeds =
				swerveController.getTargetSpeeds(vX, vY, 0, 0, currentHeading, RobotConstants.MAX_SPEED);

		translation = SwerveController.getTranslation2d(desiredSpeeds);

		swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
		logData(translation, currentYaw);
	}

	private void logData(Translation2d translation, double currentYaw) {
		Logger.recordOutput("Drive/Translation", translation.toString());
		Logger.recordOutput("Drive/Target Angle", swerve.getRotationFFController().getTargetAngle());
		Logger.recordOutput("Drive/Current Angle", currentYaw);
		Logger.recordOutput("Drive/Rot State", swerveRotationController.getState().toString());
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}

	@Override
	public void end(boolean interrupted) {
		swerve.drive(new Translation2d(), 0, true);
		swerve.setMotorBrake(false);
		swerve.getRotationFFController().setState(RotationState.NONE);
	}
}
