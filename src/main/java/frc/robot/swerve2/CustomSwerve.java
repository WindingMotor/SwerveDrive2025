// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve2;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class CustomSwerve extends SubsystemBase {
	// Robot dimensions
	private static final double kTrackWidth = Units.inchesToMeters(10.375); // meters
	private static final double kWheelBase = Units.inchesToMeters(10.375); // meters

	// Swerve module positions relative to center of robot
	private final Translation2d m_frontLeftLocation =
			new Translation2d(kWheelBase / 2, kTrackWidth / 2);
	private final Translation2d m_frontRightLocation =
			new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
	private final Translation2d m_backLeftLocation =
			new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
	private final Translation2d m_backRightLocation =
			new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

	// Swerve Modules
	private final SwerveModule m_frontLeft;
	private final SwerveModule m_frontRight;
	private final SwerveModule m_backLeft;
	private final SwerveModule m_backRight;

	// Gyro
	private final Pigeon2 m_gyro;

	// Kinematics and Odometry
	private final SwerveDriveKinematics m_kinematics;
	private final SwerveDriveOdometry m_odometry;

	public CustomSwerve() {
		// Create swerve modules with the correct IDs and offsets from your config
		m_frontLeft =
				new SwerveModule(
						5, // drive motor ID
						6, // turning motor ID
						6, // encoder ID
						195.7, // absoluteEncoderOffset
						"canivore", // CAN bus name
						"frontLeft");

		m_frontRight = new SwerveModule(1, 2, 4, 71.4, "canivore", "frontRight");

		m_backLeft = new SwerveModule(3, 4, 7, 223.5, "canivore", "backLeft");

		m_backRight = new SwerveModule(7, 8, 8, 268.1, "canivore", "backRight");

		// Initialize gyro with correct ID and CANbus
		m_gyro = new Pigeon2(13, "canivore");
		m_gyro.reset();

		m_kinematics =
				new SwerveDriveKinematics(
						m_frontLeftLocation, m_frontRightLocation,
						m_backLeftLocation, m_backRightLocation);

		m_odometry = new SwerveDriveOdometry(m_kinematics, getRotation2d(), getModulePositions());
	}

	public Command driveCommand(
			DoubleSupplier xSpeedGet, DoubleSupplier ySpeedGet, DoubleSupplier rotGet) {
		return run(
				() -> {
					// Convert to meters per second
					double xSpeed = xSpeedGet.getAsDouble() * 4; // Max speed in meters per second
					double ySpeed = ySpeedGet.getAsDouble() * 4;
					double rot = rotGet.getAsDouble() * 4;

					// Calculate module states
					var swerveModuleStates =
							m_kinematics.toSwerveModuleStates(
									ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d()));

					// Normalize wheel speeds if any speed is greater than max speed
					SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 4.0);

					// Set each module state
					setModuleStates(swerveModuleStates);
				});
	}

	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees());
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
	}

	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_backLeft.getPosition(),
			m_backRight.getPosition()
		};
	}

	public void setModuleStates(SwerveModuleState[] states) {
		m_frontLeft.setDesiredState(states[0]);
		m_frontRight.setDesiredState(states[1]);
		m_backLeft.setDesiredState(states[2]);
		m_backRight.setDesiredState(states[3]);
	}

	public void stopModules() {
		m_frontLeft.stop();
		m_frontRight.stop();
		m_backLeft.stop();
		m_backRight.stop();
	}

	@Override
	public void periodic() {
		// Update odometry
		m_odometry.update(getRotation2d(), getModulePositions());
		m_frontLeft.log();
		m_frontRight.log();
		m_backLeft.log();
		m_backRight.log();
	}
}
