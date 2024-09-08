// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;

/** Base interface for Swerve Drive IO operations. */
public interface IO_SwerveBase {

	/** AutoLogged inputs for swerve drive. */
	@AutoLog
	public static class SwerveInputs {
		public Pose2d pose = new Pose2d();
		public Rotation2d heading = new Rotation2d();
		public ChassisSpeeds velocity = new ChassisSpeeds();
	}

	/**
	 * Updates the swerve drive inputs.
	 *
	 * @param inputs The SwerveInputs object to update.
	 */
	void updateInputs(SwerveInputs inputs);

	/**
	 * Drives the robot using a Pose2d.
	 *
	 * @param pose The target pose.
	 * @param isFieldRelative Whether the drive is field-relative.
	 * @param isOpenLoop Whether to use open-loop control.
	 */
	void drive(Translation2d translation, double theta, boolean isFieldRelative, boolean isOpenLoop);

	/**
	 * Gets the maximum velocity of the swerve drive.
	 *
	 * @return The maximum velocity in meters per second.
	 */
	double getMaximumVelocity();

	/**
	 * Gets the maximum angular velocity of the swerve drive.
	 *
	 * @return The maximum angular velocity in radians per second.
	 */
	double getMaximumAngularVelocity();

	/** Updates the odometry of the swerve drive. */
	void updateOdometry();

	/**
	 * Resets the odometry to a specific pose.
	 *
	 * @param pose The pose to reset to.
	 */
	void resetOdometry(Pose2d pose);

	/**
	 * Sets the chassis speeds of the swerve drive.
	 *
	 * @param chassisSpeeds The target chassis speeds.
	 */
	void setChassisSpeeds(ChassisSpeeds chassisSpeeds);

	/**
	 * Gets the PID constants for heading control.
	 *
	 * @return The PID constants for heading.
	 */
	PIDConstants getHeadingPID();

	/**
	 * Gets the configuration radius of the swerve drive.
	 *
	 * @return The configuration radius in meters.
	 */
	double getConfigurationRadius();

	/**
	 * Sets the brake mode of the swerve drive motors.
	 *
	 * @param isBrakeMode True to enable brake mode, false for coast mode.
	 */
	void setBrakeMode(boolean isBrakeMode);

	/**
	 * Sets the current limit for the swerve drive motors.
	 *
	 * @param currentLimit The current limit in amps.
	 * @param isTurnMotors True to set limit for turn motors, false for drive motors.
	 */
	void setMotorCurrentLimit(int currentLimit, boolean isTurnMotors);
}
