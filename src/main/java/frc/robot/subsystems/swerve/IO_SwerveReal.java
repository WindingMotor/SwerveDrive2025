// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Implementation of IO_SwerveBase for real robot hardware. */
public class IO_SwerveReal implements IO_SwerveBase {

	private static final File JSON_DIRECTORY = new File(Filesystem.getDeployDirectory(), "swerve");
	private static final double MAX_MODULE_SPEED = 4.5; // meters per second

	private final SwerveDrive swerveDrive;

	/**
	 * Constructs a new IO_SwerveReal instance. Initializes the SwerveDrive with configuration from
	 * JSON files.
	 */
	public IO_SwerveReal() {
		try {
			swerveDrive = new SwerveParser(JSON_DIRECTORY).createSwerveDrive(MAX_MODULE_SPEED);
			SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
			swerveDrive.setHeadingCorrection(false);
			swerveDrive.setCosineCompensator(false);

		} catch (IOException e) {
			throw new RuntimeException("Failed to create SwerveDrive", e);
		}
	}

	/** {@inheritDoc} */
	@Override
	public void updateInputs(SwerveInputsAutoLogged inputs) {
		inputs.pose = swerveDrive.getPose();
		inputs.heading = swerveDrive.getYaw();
		inputs.velocity = swerveDrive.getRobotVelocity();
		inputs.maximumVelocity = swerveDrive.getMaximumVelocity();
		inputs.maximumAngularVelocity = swerveDrive.getMaximumAngularVelocity();
		inputs.setHeadingPID(
				new PIDConstants(
						swerveDrive.swerveController.config.headingPIDF.p,
						swerveDrive.swerveController.config.headingPIDF.i,
						swerveDrive.swerveController.config.headingPIDF.d));
		inputs.setTranslationPID(new PIDConstants(0.5, 0.0, 0.0));
		inputs.configurationRadius = swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters();
	}

	/** {@inheritDoc} */
	@Override
	public void resetOdometry(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}

	/** {@inheritDoc} */
	@Override
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		swerveDrive.setChassisSpeeds(chassisSpeeds);
	}

	/** {@inheritDoc} */
	@Override
	public void drive(
			Translation2d translation, double theta, boolean isFieldRelative, boolean isOpenLoop) {
		swerveDrive.drive(translation, theta, isFieldRelative, isOpenLoop);
	}

	/** {@inheritDoc} */
	@Override
	public void updateOdometry() {
		swerveDrive.updateOdometry();
	}

	/** {@inheritDoc} */
	@Override
	public void setMotorCurrentLimit(int currentLimit, boolean isTurnMotors) {
		for (SwerveModule module : swerveDrive.getModules()) {
			if (isTurnMotors) {
				module.getAngleMotor().setCurrentLimit(currentLimit);
			} else {
				module.getDriveMotor().setCurrentLimit(currentLimit);
			}
		}
	}

	/** {@inheritDoc} */
	@Override
	public void setBrakeMode(boolean isBrakeMode) {
		for (SwerveModule module : swerveDrive.swerveDriveConfiguration.modules) {
			module.setMotorBrake(isBrakeMode);
		}
	}
}
