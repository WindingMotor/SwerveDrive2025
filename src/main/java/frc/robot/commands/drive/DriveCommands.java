// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.math.ExpDecayFF;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Commands for controlling the drive subsystem. */
public class DriveCommands {
	// #region Constants
	private static final double DEADBAND = 0.01;
	private static final double MAX_ASSIST_DISTANCE = 2.0; // Meters

	private static final ExpDecayFF rotationController = new ExpDecayFF(6.0, 1.0, 0.25);

	// #endregion

	// Private constructor to prevent instantiation
	private DriveCommands() {}

	// #region Helper Methods
	/**
	 * Converts joystick inputs to a linear velocity vector.
	 *
	 * @param x X-axis joystick input
	 * @param y Y-axis joystick input
	 * @return Translation2d representing linear velocity
	 */
	private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
		// Apply deadband
		double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
		Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

		// Square magnitude for more precise control
		linearMagnitude = linearMagnitude * linearMagnitude;

		// Return new linear velocity
		return new Pose2d(new Translation2d(), linearDirection)
				.transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
				.getTranslation();
	}

	// #endregion

	// #region Basic Drive Commands
	/**
	 * Field relative drive command using two joysticks (controlling linear and angular velocities).
	 *
	 * @param drive Drive subsystem
	 * @param xSupplier X-axis joystick input supplier
	 * @param ySupplier Y-axis joystick input supplier
	 * @param omegaSupplier Rotation joystick input supplier
	 * @return Command for manual driving
	 */
	public static Command driveNormal(
			Drive drive,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier omegaSupplier) {
		return Commands.run(
				() -> {
					// Get linear velocity
					Translation2d linearVelocity =
							getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

					// Apply rotation deadband
					double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

					// Convert to field relative speeds & send command
					ChassisSpeeds speeds =
							new ChassisSpeeds(
									linearVelocity.getX() * 5.0,
									linearVelocity.getY() * 5.0,
									omega * drive.getMaxAngularSpeedRadPerSec());

					drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
				},
				drive);
	}

	// #endregion

	// #region Assisted Drive Commands
	/**
	 * Field relative drive command with rotation assist. When rotation assist is enabled, the robot
	 * will automatically rotate to the nearest zone of the field.
	 *
	 * @param drive Drive subsystem
	 * @param xSupplier X-axis joystick input supplier
	 * @param ySupplier Y-axis joystick input supplier
	 * @param omegaSupplier Rotation joystick input supplier
	 * @param assistOnSupplier Boolean supplier for whether assist is enabled
	 * @return Command for assisted driving
	 */
	public static Command driveWithAssist(
			Drive drive,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier omegaSupplier,
			BooleanSupplier assistOnSupplier) {

		// Construct command
		return Commands.run(
				() -> {
					// Get target rotation based on closest AprilTag
					int closestTagId = drive.getRecentClosestTagData().getFirst();
					double distanceM = drive.getRecentClosestTagData().getSecond();
					ZoneAngle targetZone = getZoneAngleForTagID(closestTagId);

					// Convert zone angle to radians
					Rotation2d targetRotation = Rotation2d.fromDegrees(targetZone.getAngle());

					// Get linear velocity
					Translation2d linearVelocity =
							getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

					// Calculate angular speed based on assist mode and distance check
					double omega = 0.0;
					if (assistOnSupplier.getAsBoolean()
							&& distanceM < MAX_ASSIST_DISTANCE
							&& targetZone != ZoneAngle.NONE) {
						omega =
								rotationController.calculate(
												drive.getRotation().getDegrees(), targetRotation.getDegrees())
										+ (omegaSupplier.getAsDouble());
					} else {
						omega =
								MathUtil.applyDeadband(
										omegaSupplier.getAsDouble() * drive.getMaxAngularSpeedRadPerSec(), DEADBAND);
					}

					// Convert to field relative speeds & send command
					ChassisSpeeds speeds =
							new ChassisSpeeds(
									linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
									linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
									omega);

					// Command the drive
					drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));

					// Log target rotation for debugging
					Logger.recordOutput("Drive/TargetRotation", targetRotation.getDegrees());
				},
				drive);
	}

	// #endregion

	// #region Autonomous Drive Commands
	/**
	 * Creates a command that will drive to a specified pose using PID control. Uses separate PID
	 * controllers for x, y and rotation.
	 *
	 * @param drive Drive subsystem
	 * @param targetPoseSupplier Supplier for the target pose
	 * @return Command for driving to a pose
	 */
	public static Command driveToPose(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
		// Create ExpDecayFF controllers for x, y and rotation
		ExpDecayFF xController = new ExpDecayFF(210.0, 1.5, 0.04);
		ExpDecayFF yController = new ExpDecayFF(210.0, 1.5, 0.04);
		ExpDecayFF rotController = new ExpDecayFF(6, 1.0, 0.95);

		return Commands.run(
						() -> {
							// Get current pose and target pose
							Pose2d currentPose = drive.getPose();
							Pose2d targetPose = targetPoseSupplier.get();

							// Calculate control outputs
							double xOutput = xController.calculate(currentPose.getX(), targetPose.getX());
							double yOutput = yController.calculate(currentPose.getY(), targetPose.getY());
							double rotationOutput =
									rotController.calculate(
											currentPose.getRotation().getDegrees(),
											targetPose.getRotation().getDegrees());

							// Create field-relative speeds
							ChassisSpeeds speeds =
									ChassisSpeeds.fromFieldRelativeSpeeds(
											xOutput, yOutput, rotationOutput, drive.getRotation());

							// Command the drive
							drive.runVelocity(speeds);

							// Log target pose for visualization
							Logger.recordOutput("ZonePose/TargetPose", targetPose);
							Logger.recordOutput("ZonePose/ErrorX", currentPose.getX() - targetPose.getX());
							Logger.recordOutput("ZonePose/ErrorY", currentPose.getY() - targetPose.getY());
							Logger.recordOutput(
									"ZonePose/ErrorOmega",
									currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees());
							Logger.recordOutput(
									"ZonePose/XAtTargret?",
									xController.atTarget(currentPose.getX(), targetPose.getX()));
							Logger.recordOutput(
									"ZonePose/YAtTargret?",
									yController.atTarget(currentPose.getY(), targetPose.getY()));
							Logger.recordOutput(
									"ZonePose/OmegaAtTargret?",
									rotController.atTarget(
											currentPose.getRotation().getDegrees(),
											targetPose.getRotation().getDegrees()));
						},
						drive)
				.until(
						() -> {
							Pose2d currentPose = drive.getPose();
							Pose2d targetPose = targetPoseSupplier.get();

							return xController.atTarget(currentPose.getX(), targetPose.getX())
									&& yController.atTarget(currentPose.getY(), targetPose.getY())
									&& rotController.atTarget(
											currentPose.getRotation().getDegrees(),
											targetPose.getRotation().getDegrees());
						});
	}

	/**
	 * Creates a command that will drive to a specified zone pose.
	 *
	 * @param drive Drive subsystem
	 * @param zonePose The zone pose to drive to
	 * @return Command for driving to a zone pose
	 */
	public static Command driveToZone(Drive drive, ZonePose zonePose) {
		return driveToPose(drive, zonePose.getPose());
	}

	/**
	 * Overloaded version that accepts a fixed target pose rather than a supplier.
	 *
	 * @param drive Drive subsystem
	 * @param targetPose The target pose
	 * @return Command for driving to a pose
	 */
	public static Command driveToPose(Drive drive, Pose2d targetPose) {
		return driveToPose(drive, () -> targetPose);
	}

	/**
	 * Gets the closest source pose based on the closest AprilTag.
	 *
	 * @param drive Drive subsystem
	 * @return Optional containing the closest source pose, or empty if no valid tag is found
	 */
	public static Optional<Pose2d> getCloserSourcePose(Drive drive) {
		int id = drive.getRecentClosestTagData().getFirst();

		if (id == 1) {
			return Optional.ofNullable(
					new Pose2d(new Translation2d(16.94, 1.2), Rotation2d.fromDegrees(-55)));
		} else if (id == 2) {
			return Optional.ofNullable(
					new Pose2d(new Translation2d(16.31, 7.35), Rotation2d.fromDegrees(44)));
		} else {
			return Optional.empty();
		}
	}

	/**
	 * Creates a command that will drive to the closest source based on AprilTag detection.
	 *
	 * @param drive Drive subsystem
	 * @return Command for driving to the source
	 */
	public static Command driveToSource(Drive drive) {
		Optional<Pose2d> poseOptional = getCloserSourcePose(drive);
		if (poseOptional.isPresent()) {
			return DriveCommands.driveToPose(drive, poseOptional.get());
		} else {
			return new PrintCommand("Drive to Source: Empty Optional");
		}
	}

	// #endregion

	// #region Zone Angle Definitions
	public enum ZoneAngle {
		NONE(0),
		FORWARD(180),
		BACKWARD(0),
		RIGHT(-90),
		LEFT(90),
		SOURCE_RIGHT(46.9),
		SOURCE_LEFT(-46.9),
		REEF_BOTTOM_RIGHT(-120),
		REEF_BOTTOM_LEFT(120),
		REEF_BOTTOM(180),
		REEF_TOP_RIGHT(-60),
		REEF_TOP_LEFT(60),
		REEF_TOP(0);

		private final double angle;

		ZoneAngle(double angle) {
			this.angle = angle;
		}

		public double getAngle() {
			return angle;
		}

		public Rotation2d getRotation() {
			return Rotation2d.fromDegrees(angle);
		}
	}

	/**
	 * Gets the appropriate zone angle based on AprilTag ID and alliance color.
	 *
	 * @param id The AprilTag ID
	 * @return The ZoneAngle for the given tag ID
	 */
	public static ZoneAngle getZoneAngleForTagID(int id) {
		switch (id) {
			case 2:
			case 12:
				return ZoneAngle.SOURCE_RIGHT;

			case 1:
			case 13:
				return ZoneAngle.SOURCE_LEFT;

			case 7:
			case 18:
				return ZoneAngle.REEF_BOTTOM;

			case 6:
			case 19:
				return ZoneAngle.REEF_BOTTOM_LEFT;

			case 8:
			case 17:
				return ZoneAngle.REEF_BOTTOM_RIGHT;

			case 11:
			case 20:
				return ZoneAngle.REEF_TOP_LEFT;

			case 10:
			case 21:
				return ZoneAngle.REEF_TOP;

			case 9:
			case 22:
				return ZoneAngle.REEF_TOP_RIGHT;

			default:
				return ZoneAngle.NONE;
		}
	}

	// #endregion

	// #region Zone Pose Definitions
	public enum ZonePose {
		NONE(new Translation2d(), ZoneAngle.NONE),
		FORWARD(new Translation2d(), ZoneAngle.FORWARD),
		BACKWARD(new Translation2d(), ZoneAngle.BACKWARD),
		RIGHT(new Translation2d(), ZoneAngle.RIGHT),
		LEFT(new Translation2d(), ZoneAngle.LEFT),

		// SOURCE
		SOURCE_RIGHT(new Translation2d(15.72, 7.7), ZoneAngle.SOURCE_RIGHT),
		SOURCE_LEFT(new Translation2d(16.8, 0.95), ZoneAngle.SOURCE_LEFT),

		// BOTTOM RIGHT
		REEF_BOTTOM_RIGHT_TOP(new Translation2d(13.51, 5.14), ZoneAngle.REEF_BOTTOM_RIGHT), // g
		REEF_BOTTOM_RIGHT_BOTTOM(new Translation2d(13.8, 4.98), ZoneAngle.REEF_BOTTOM_RIGHT), // g

		// BOTTOM LEFT
		// REEF_BOTTOM_LEFT(new Translation2d(), ZoneAngle.REEF_BOTTOM_LEFT),

		// BOTTOM
		REEF_BOTTOM_LEFT(new Translation2d(14.384, 3.852), ZoneAngle.REEF_BOTTOM),
		REEF_BOTTOM_RIGHT(new Translation2d(14.384, 4.181), ZoneAngle.REEF_BOTTOM),

		// TOP RIGHT
		REEF_TOP_RIGHT_BOTTOM(new Translation2d(12.58, 5.18), ZoneAngle.REEF_TOP_RIGHT),
		REEF_TOP_RIGHT_TOP(new Translation2d(12.31, 4.99), ZoneAngle.REEF_TOP_RIGHT),

		// TOP LEFT
		REEF_TOP_LEFT(new Translation2d(), ZoneAngle.REEF_TOP_LEFT),

		REEF_TOP(new Translation2d(), ZoneAngle.REEF_TOP);

		private final Translation2d translation;
		private final ZoneAngle zoneAngle;

		ZonePose(Translation2d translation, ZoneAngle zoneAngle) {
			this.translation = translation;
			this.zoneAngle = zoneAngle;
		}

		public Translation2d getTranslation() {
			return translation;
		}

		public ZoneAngle getZoneAngle() {
			return zoneAngle;
		}

		public Pose2d getPose() {
			return new Pose2d(translation, zoneAngle.getRotation());
		}
	}
	// #endregion
}
