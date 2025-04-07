// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.PoseAllignment;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.math.AllianceFlipUtil;
import frc.robot.util.math.ExpDecayFF;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
	private static final double DEADBAND = 0.01;
	private static final double ANGLE_MAX_VELOCITY = 10.0;
	private static final double ANGLE_MAX_ACCELERATION = 15.0;
	private static final double FF_START_DELAY = 2.0; // Secs
	private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
	private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
	private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
	private static final double MAX_ASSIST_DISTANCE = 2.0; // Meters

	private static final double ANGLE_KP = 5.5;
	private static final double ANGLE_KD = 0.6;

	private static final double TRANSLATION_KP = 4.5;
	private static final double TRANSLATION_KD = 0.1;

	private static final ExpDecayFF rotationController = new ExpDecayFF(6.5, 1.5, 0.25);

	private DriveCommands() {}

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

	/**
	 * Field relative drive command using two joysticks (controlling linear and angular velocities).
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

					// Square rotation value for more precise control
					// omega = Math.copySign(omega * omega, omega);

					// Convert to field relative speeds & send command
					ChassisSpeeds speeds =
							new ChassisSpeeds(
									linearVelocity.getX() * 5.0,
									linearVelocity.getY() * 5.0,
									omega * drive.getMaxAngularSpeedRadPerSec());
					/*
					boolean isFlipped =
							DriverStation.getAlliance().isPresent()
									&& DriverStation.getAlliance().get() == Alliance.Red;
					*/
					drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
				},
				drive);
	}

	/**
	 * Creates a command that will drive to a specified pose using PID control. Uses separate PID
	 * controllers for x, y and rotation.
	 */
	public static Command driveToPose(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
		// Create ExpDecayFF controllers for x, y and rotation
		ExpDecayFF xController = new ExpDecayFF(200.0, 1.5, 0.041);
		ExpDecayFF yController = new ExpDecayFF(200.0, 1.5, 0.041);
		ExpDecayFF rotController = new ExpDecayFF(6, 1.0, 1.0);

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
	 * Creates a command that will drive to a specified pose using a vector-based approach with motion
	 * profiling for smoother, more direct paths.
	 *
	 * @param drive The drive subsystem
	 * @param targetPoseSupplier Supplier for the target pose
	 * @return A command that will drive to the target pose
	 */
	public static Command driveToPoseVector(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
		// Constants for profiled motion control
		final double MAX_TRANSLATION_SPEED = 3.0; // meters per second
		final double MAX_TRANSLATION_ACCEL = 2.5; // meters per second squared
		final double MAX_ROTATION_SPEED = Math.PI; // radians per second
		final double MAX_ROTATION_ACCEL = 4 * Math.PI; // radians per second squared

		// Constants for feedforward scaling
		final double FF_MIN_RADIUS = 0.2; // meters
		final double FF_MAX_RADIUS = 0.6; // meters

		// Position and velocity tolerances
		final double TRANSLATION_TOLERANCE = 0.041; // meters
		final double HEADING_TOLERANCE = Units.degreesToRadians(1.5); // radians
		final double VELOCITY_TOLERANCE = 0.075; // m/s
		final double ROTATION_VELOCITY_TOLERANCE = Math.PI / 16; // rad/s

		// Create controllers with constraints
		ProfiledPIDController translationController =
				new ProfiledPIDController(
						6.25,
						0.0,
						0.0,
						new TrapezoidProfile.Constraints(MAX_TRANSLATION_SPEED, MAX_TRANSLATION_ACCEL));

		ProfiledPIDController headingController =
				new ProfiledPIDController(
						6.5,
						0.0,
						0.0,
						new TrapezoidProfile.Constraints(MAX_ROTATION_SPEED, MAX_ROTATION_ACCEL));

		// Set tolerances
		translationController.setTolerance(TRANSLATION_TOLERANCE, VELOCITY_TOLERANCE);
		headingController.setTolerance(HEADING_TOLERANCE, ROTATION_VELOCITY_TOLERANCE);
		headingController.enableContinuousInput(-Math.PI, Math.PI);

		return Commands.run(
						() -> {
							// Get current pose and target pose
							Pose2d currentPose = drive.getPose();
							Pose2d targetPose = targetPoseSupplier.get();

							// Calculate distance to target
							double currentDistance =
									currentPose.getTranslation().getDistance(targetPose.getTranslation());

							// Scale feedforward based on distance
							double ffScaler =
									MathUtil.clamp(
											(currentDistance - FF_MIN_RADIUS) / (FF_MAX_RADIUS - FF_MIN_RADIUS),
											0.0,
											1.0);

							// Calculate translation control output
							double driveVelocityScalar =
									translationController.getSetpoint().velocity * ffScaler
											+ translationController.calculate(currentDistance, 0.0);

							// If close enough to target, stop translation
							if (currentDistance < translationController.getPositionTolerance()) {
								driveVelocityScalar = 0.0;
							}

							// Calculate heading control output
							double headingError =
									getShortestAngleDifference(
											currentPose.getRotation().getRadians(),
											targetPose.getRotation().getRadians());

							double headingVelocity =
									headingController.getSetpoint().velocity * ffScaler
											+ headingController.calculate(
													currentPose.getRotation().getRadians(),
													targetPose.getRotation().getRadians());

							// If close enough to target heading, stop rotation
							if (Math.abs(headingError) < headingController.getPositionTolerance()) {
								headingVelocity = 0.0;
							}

							// Calculate direction vector to target
							Rotation2d directionToTarget =
									targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle();

							// Convert velocity scalar to vector in direction of target
							Translation2d driveVelocity =
									new Translation2d(
											driveVelocityScalar * directionToTarget.getCos(),
											driveVelocityScalar * directionToTarget.getSin());

							// Create field-relative speeds
							ChassisSpeeds speeds =
									ChassisSpeeds.fromFieldRelativeSpeeds(
											driveVelocity.getX(),
											driveVelocity.getY(),
											headingVelocity,
											drive.getRotation());

							// Command the drive
							drive.runVelocity(speeds);

							// Logging similar to original code
							Logger.recordOutput("VectorDrive/TargetPose", targetPose);
							Logger.recordOutput("VectorDrive/CurrentDistance", currentDistance);
							Logger.recordOutput("VectorDrive/TranslationOutput", driveVelocityScalar);
							Logger.recordOutput("VectorDrive/RotationOutput", headingVelocity);
							Logger.recordOutput("VectorDrive/FFScaler", ffScaler);
							Logger.recordOutput("VectorDrive/HeadingError", headingError);
							Logger.recordOutput(
									"VectorDrive/AtTranslationTarget",
									currentDistance < translationController.getPositionTolerance());
							Logger.recordOutput(
									"VectorDrive/AtHeadingTarget",
									Math.abs(headingError) < headingController.getPositionTolerance());
							Logger.recordOutput("VectorDrive/DirectionToTarget", directionToTarget.getDegrees());
						},
						drive)
				.until(
						() -> {
							Pose2d currentPose = drive.getPose();
							Pose2d targetPose = targetPoseSupplier.get();

							double currentDistance =
									currentPose.getTranslation().getDistance(targetPose.getTranslation());
							double headingError =
									getShortestAngleDifference(
											currentPose.getRotation().getRadians(),
											targetPose.getRotation().getRadians());

							boolean atTranslationTarget =
									currentDistance < translationController.getPositionTolerance()
											&& Math.abs(translationController.getSetpoint().velocity)
													< VELOCITY_TOLERANCE;

							boolean atHeadingTarget =
									Math.abs(headingError) < headingController.getPositionTolerance()
											&& Math.abs(headingController.getSetpoint().velocity)
													< ROTATION_VELOCITY_TOLERANCE;

							return atTranslationTarget && atHeadingTarget;
						});
	}

	/** Calculates the shortest angular distance between two angles in radians. */
	private static double getShortestAngleDifference(double from, double to) {
		double error = to - from;

		// Normalize to -PI to PI
		while (error > Math.PI) error -= 2 * Math.PI;
		while (error < -Math.PI) error += 2 * Math.PI;

		return error;
	}

	public static Command driveAlign(
			Drive drive,
			Supplier<ZonePose> zonePose,
			// Supplier<Boolean> isRed,
			CommandXboxController driverController,
			DoubleSupplier elevatorHeightMeters) {

		// Create ExpDecayFF controllers for x, y and rotation
		ExpDecayFF xController = new ExpDecayFF(200.0, 1.5, 0.048);
		ExpDecayFF yController = new ExpDecayFF(200.0, 1.5, 0.048);
		ExpDecayFF rotController = new ExpDecayFF(6, 1.0, 1.0);

		return Commands.run(
						() -> {

							// Check if isRed is null, if the field is stupid....
							/*
							if (isRed == null) {
								// Logger.recordMetadata("Auto Align NULL Trip", "TRUE");
								DriverStation.reportError("Auto Align Null", true);
								return;
							}*/

							var alli = DriverStation.getAlliance();

							if (!alli.isPresent()) {
								return;
							}

							boolean isRed = false;

							if (alli.get() == Alliance.Red) {
								isRed = true;

							} else if (alli.get() == Alliance.Blue) {
								isRed = false;

							} else {
								// Return if the alli does not exist. Ends the CMD
								return;
							}

							// Zone Pose Get
							Optional<Pose2d> adjustedPose = zonePose.get().getPoseForAlliance(isRed);
							if (adjustedPose.isEmpty()) {
								return;
							}

							// Get current pose and target pose
							Pose2d currentPose = drive.getPose();
							Pose2d targetPose = adjustedPose.get();

							// Calculate control outputs
							double xOutput = xController.calculate(currentPose.getX(), targetPose.getX());
							double yOutput = yController.calculate(currentPose.getY(), targetPose.getY());
							double rotationOutput =
									rotController.calculate(
											currentPose.getRotation().getDegrees(),
											targetPose.getRotation().getDegrees());

							// Calculate speed scaling factor based on elevator height
							double height = elevatorHeightMeters.getAsDouble();
							double speedScaleFactor = 1.0;

							// If height is greater than 0.92m, start scaling down the speed
							if (height > 0.92) {
								// Linear scaling from 1.0 at 0.92m to 0.3 at 1.65m (max height)
								// This provides significant speed reduction at max height to prevent tipping
								speedScaleFactor = 1.0 - (0.7 * Math.min(1.0, (height - 0.92) / (1.65 - 0.92)));
							}

							// Apply scaling factor to all outputs
							xOutput *= speedScaleFactor;
							yOutput *= speedScaleFactor;
							rotationOutput *= speedScaleFactor;

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
							Logger.recordOutput("ZonePose/SpeedScaleFactor", speedScaleFactor);
							Logger.recordOutput("ZonePose/ElevatorHeight", height);
						},
						drive)
				.until(
						() -> {
							if (driverController.button(2).getAsBoolean()) {
								return true;
							}

							var alli = DriverStation.getAlliance();

							if (!alli.isPresent()) {
								return true;
							}

							boolean isRed = false;

							if (alli.get() == Alliance.Red) {
								isRed = true;

							} else if (alli.get() == Alliance.Blue) {
								isRed = false;

							} else {
								// Return if the alli does not exist. Ends the CMD
								return true;
							}

							Optional<Pose2d> adjustedPose = zonePose.get().getPoseForAlliance(isRed);
							if (adjustedPose.isEmpty()) {
								return true;
							}

							Pose2d currentPose = drive.getPose();
							Pose2d targetPose = adjustedPose.get();

							return xController.atTarget(currentPose.getX(), targetPose.getX())
									&& yController.atTarget(currentPose.getY(), targetPose.getY())
									&& rotController.atTarget(
											currentPose.getRotation().getDegrees(),
											targetPose.getRotation().getDegrees());
						});
	}

	// Drive to a predefined zone based on zonePose alliance adjustment
	/*
	public static Command driveToZone(Drive drive, Supplier<ZonePose> zonePose) {

		var pose = zonePose.get().getPoseForAlliance();
		if (pose.isPresent()) {
			return driveToPose(drive, pose.get());
		} else {
			return new PrintCommand("Drive to Zone: Empty Optional");
		}
	}
		*/

	/** Overloaded version that accepts a fixed target pose rather than a supplier. */
	public static Command driveToPose(Drive drive, Pose2d targetPose) {
		return driveToPose(drive, () -> targetPose);
	}

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

	public static Command driveToSource(Drive drive) {
		Optional<Pose2d> poseOptional = getCloserSourcePose(drive);
		if (poseOptional.isPresent()) {
			return DriveCommands.driveToPose(drive, poseOptional.get());
		} else {
			return new PrintCommand("Drive to Source: Empty Optional");
		}
	}

	/*
	 * Field relative drive command using two joysticks (controlling linear and angular velocities).
	 * Includes rotation assist mode. When rotation assist is enabled, the robot will automatically
	 * rotate to the nearest ZONE of the field.
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

					// Get allaince
					var alliance = DriverStation.getAlliance();

					// Get target rotation based on closest AprilTag
					int closestTagId = drive.getRecentClosestTagData().getFirst();
					double distanceM = drive.getRecentClosestTagData().getSecond();
					ZoneAngle targetZone = getZoneAngleForTagID(closestTagId);

					// Convert zone angle to radians
					Rotation2d initalTargetRotation = Rotation2d.fromDegrees(targetZone.getAngle());
					Rotation2d targetRotation = null;

					// If we are on blue alliance flip target rotation by 180
					if (alliance.isPresent()) {
						if (alliance.get() == Alliance.Blue) {
							targetRotation = Rotation2d.fromDegrees(initalTargetRotation.getDegrees() - 180);
							Logger.recordOutput("Drive/FlipTargetrot", true);
						} else {
							targetRotation = initalTargetRotation;
							Logger.recordOutput("Drive/FlipTargetrot", false);
						}
					} else {
						targetRotation = new Rotation2d();
					}

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
					if (alliance.isPresent()) {
						if (alliance.get() == Alliance.Red) {
							// Red
							drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
							Logger.recordOutput("Drive/ISROT", false);
						} else {
							// Blue
							drive.runVelocity(
									ChassisSpeeds.fromFieldRelativeSpeeds(
											speeds, drive.getRotation().plus(Rotation2d.fromDegrees(180))));
							Logger.recordOutput("Drive/ISROT", true);
						}
					} else {
						// Backup if there is no alliance. I hope i dont have to drive backwards. (Isaac)
						drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
					}

					// Log target rotation for debugging
					Logger.recordOutput("Drive/TargetRotation", targetRotation.getDegrees());
				},
				drive);

		// Reset PID controller when command starts

	}

	/**
	 * Creates a command that will drive to a specified transform and zone angle using PID control.
	 * The transform is relative to the robot's current pose.
	 */
	public static Command driveToTransform(Drive drive, Transform2d transform, ZoneAngle zoneAngle) {
		// Create PID controllers for x, y and rotation
		ProfiledPIDController xController =
				new ProfiledPIDController(
						TRANSLATION_KP,
						0.0,
						TRANSLATION_KD,
						new TrapezoidProfile.Constraints(
								drive.getMaxLinearSpeedMetersPerSec(), 2.0)); // Max acceleration in m/s^2

		ProfiledPIDController yController =
				new ProfiledPIDController(
						TRANSLATION_KP,
						0.0,
						TRANSLATION_KD,
						new TrapezoidProfile.Constraints(
								drive.getMaxLinearSpeedMetersPerSec(), 2.0)); // Max acceleration in m/s^2

		ProfiledPIDController rotationController =
				new ProfiledPIDController(
						ANGLE_KP,
						0.0,
						ANGLE_KD,
						new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
		rotationController.enableContinuousInput(-Math.PI, Math.PI);

		return Commands.run(
						() -> {
							// Get current pose and calculate target pose
							Pose2d currentPose = drive.getPose();
							Pose2d targetPose = currentPose.transformBy(transform);

							// Get target rotation from zone angle
							Rotation2d targetRotation = Rotation2d.fromDegrees(zoneAngle.getAngle());

							// Calculate control outputs
							double xOutput = xController.calculate(currentPose.getX(), targetPose.getX());
							double yOutput = yController.calculate(currentPose.getY(), targetPose.getY());
							double rotationOutput =
									rotationController.calculate(
											currentPose.getRotation().getRadians(), targetRotation.getRadians());

							// Create field-relative speeds
							ChassisSpeeds speeds =
									ChassisSpeeds.fromFieldRelativeSpeeds(
											xOutput, yOutput, rotationOutput, drive.getRotation());

							// Command the drive
							drive.runVelocity(speeds);

							// Log data for debugging
							Logger.recordOutput("Odometry/TargetPose", targetPose);
							Logger.recordOutput("Drive/TargetRotation", targetRotation.getDegrees());
						},
						drive)
				// Reset PID controllers when command starts
				.beforeStarting(
						() -> {
							Pose2d currentPose = drive.getPose();
							xController.reset(currentPose.getX());
							yController.reset(currentPose.getY());
							rotationController.reset(currentPose.getRotation().getRadians());
						});
	}

	/** Overloaded version that accepts a transform supplier for dynamic transforms. */
	public static Command driveToTransform(
			Drive drive, Supplier<Transform2d> transformSupplier, ZoneAngle zoneAngle) {
		return driveToTransform(drive, transformSupplier.get(), zoneAngle);
	}

	/**
	 * Overloaded version that accepts both transform and zone angle suppliers for dynamic updates.
	 */
	public static Command driveToTransform(
			Drive drive, Supplier<Transform2d> transformSupplier, Supplier<ZoneAngle> zoneAngleSupplier) {
		return driveToTransform(drive, transformSupplier.get(), zoneAngleSupplier.get());
	}

	/**
	 * Measures the velocity feedforward constants for the drive motors.
	 *
	 * <p>This command should only be used in voltage control mode.
	 */
	public static Command feedforwardCharacterization(Drive drive) {
		List<Double> velocitySamples = new LinkedList<>();
		List<Double> voltageSamples = new LinkedList<>();
		Timer timer = new Timer();

		return Commands.sequence(
				// Reset data
				Commands.runOnce(
						() -> {
							velocitySamples.clear();
							voltageSamples.clear();
						}),

				// Allow modules to orient
				Commands.run(
								() -> {
									drive.runCharacterization(0.0);
								},
								drive)
						.withTimeout(FF_START_DELAY),

				// Start timer
				Commands.runOnce(timer::restart),

				// Accelerate and gather data
				Commands.run(
								() -> {
									double voltage = timer.get() * FF_RAMP_RATE;
									drive.runCharacterization(voltage);
									velocitySamples.add(drive.getFFCharacterizationVelocity());
									voltageSamples.add(voltage);
								},
								drive)

						// When cancelled, calculate and print results
						.finallyDo(
								() -> {
									int n = velocitySamples.size();
									double sumX = 0.0;
									double sumY = 0.0;
									double sumXY = 0.0;
									double sumX2 = 0.0;
									for (int i = 0; i < n; i++) {
										sumX += velocitySamples.get(i);
										sumY += voltageSamples.get(i);
										sumXY += velocitySamples.get(i) * voltageSamples.get(i);
										sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
									}
									double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
									double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

									NumberFormat formatter = new DecimalFormat("#0.00000");
									System.out.println("********** Drive FF Characterization Results **********");
									System.out.println("\tkS: " + formatter.format(kS));
									System.out.println("\tkV: " + formatter.format(kV));
								}));
	}

	/** Measures the robot's wheel radius by spinning in a circle. */
	public static Command wheelRadiusCharacterization(Drive drive) {
		SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
		WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

		return Commands.parallel(
				// Drive control sequence
				Commands.sequence(
						// Reset acceleration limiter
						Commands.runOnce(
								() -> {
									limiter.reset(0.0);
								}),

						// Turn in place, accelerating up to full speed
						Commands.run(
								() -> {
									double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
									drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
								},
								drive)),

				// Measurement sequence
				Commands.sequence(
						// Wait for modules to fully orient before starting measurement
						Commands.waitSeconds(1.0),

						// Record starting measurement
						Commands.runOnce(
								() -> {
									state.positions = drive.getWheelRadiusCharacterizationPositions();
									state.lastAngle = drive.getRotation();
									state.gyroDelta = 0.0;
								}),

						// Update gyro delta
						Commands.run(
										() -> {
											var rotation = drive.getRotation();
											state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
											state.lastAngle = rotation;
										})

								// When cancelled, calculate and print results
								.finallyDo(
										() -> {
											double[] positions = drive.getWheelRadiusCharacterizationPositions();
											double wheelDelta = 0.0;
											for (int i = 0; i < 4; i++) {
												wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
											}
											double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

											NumberFormat formatter = new DecimalFormat("#0.000");
											System.out.println(
													"********** Wheel Radius Characterization Results **********");
											System.out.println(
													"\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
											System.out.println(
													"\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
											Logger.recordOutput("Wheel Radius M ", formatter.format(wheelRadius));
											System.out.println(
													"\tWheel Radius: "
															+ formatter.format(wheelRadius)
															+ " meters, "
															+ formatter.format(Units.metersToInches(wheelRadius))
															+ " inches");
										})));
	}

	private static class WheelRadiusCharacterizationState {
		double[] positions = new double[4];
		Rotation2d lastAngle = new Rotation2d();
		double gyroDelta = 0.0;
	}

	public enum ZoneAngle {
		NONE(0),
		FORWARD(180),
		BACKWARD(0),
		RIGHT(-90),
		LEFT(90),
		SOURCE_RIGHT(53),
		SOURCE_LEFT(-55),
		REEF_BOTTOM_RIGHT(-120),
		REEF_BOTTOM_LEFT(120),
		REEF_BOTTOM(180),
		REEF_TOP_RIGHT(-60),
		REEF_TOP_LEFT(60),
		PROCESSOR(90),
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

			case 3:
			case 16:
				return ZoneAngle.PROCESSOR;

			default:
				return ZoneAngle.NONE;
		}
	}

	public enum ZonePose {
		NONE(new Translation2d(), ZoneAngle.NONE),
		FORWARD(new Translation2d(), ZoneAngle.FORWARD),
		BACKWARD(new Translation2d(), ZoneAngle.BACKWARD),
		RIGHT(new Translation2d(), ZoneAngle.RIGHT),
		LEFT(new Translation2d(), ZoneAngle.LEFT),
		PROCESSOR(new Translation2d(), ZoneAngle.PROCESSOR),

		// SOURCE
		SOURCE_RIGHT(new Translation2d(16.25, 7.25), ZoneAngle.SOURCE_RIGHT),
		SOURCE_LEFT(new Translation2d(16.8, 0.95), ZoneAngle.SOURCE_LEFT),

		// BOTTOM RIGHT
		REEF_BOTTOM_RIGHT_TOP(new Translation2d(13.559, 5.224), ZoneAngle.REEF_BOTTOM_RIGHT),
		REEF_BOTTOM_RIGHT_BOTTOM(new Translation2d(13.835, 5.055), ZoneAngle.REEF_BOTTOM_RIGHT),

		// BOTTOM LEFT
		REEF_BOTTOM_LEFT_TOP(new Translation2d(13.550, 2.856), ZoneAngle.REEF_BOTTOM_LEFT),
		REEF_BOTTOM_LEFT_BOTTOM(new Translation2d(13.856, 3), ZoneAngle.REEF_BOTTOM_LEFT),

		// BOTTOM
		REEF_BOTTOM_LEFT(new Translation2d(14.384, 3.852), ZoneAngle.REEF_BOTTOM),
		REEF_BOTTOM_RIGHT(new Translation2d(14.384, 4.181), ZoneAngle.REEF_BOTTOM),

		// TOP RIGHT
		REEF_TOP_RIGHT_BOTTOM(new Translation2d(12.558, 5.201), ZoneAngle.REEF_TOP_RIGHT),
		REEF_TOP_RIGHT_TOP(new Translation2d(12.279, 5.050), ZoneAngle.REEF_TOP_RIGHT),

		// TOP LEFT
		REEF_TOP_LEFT_TOP(new Translation2d(12.312, 3.052), ZoneAngle.REEF_TOP_LEFT),
		REEF_TOP_LEFT_BOTTOM(new Translation2d(12.559, 2.849), ZoneAngle.REEF_TOP_LEFT),

		// TOP
		REEF_TOP_LEFT(new Translation2d(11.78, 3.871), ZoneAngle.REEF_TOP),
		REEF_TOP_RIGHT(new Translation2d(11.78, 4.195), ZoneAngle.REEF_TOP);

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

		public Optional<Pose2d> getPoseForAlliance(boolean isRed) {

			// If red
			if (isRed) {
				// Red
				return Optional.of(new Pose2d(translation, zoneAngle.getRotation()));
			}

			// If not red aka blue
			if (!isRed) {
				// Blue
				Pose2d flippedPose = AllianceFlipUtil.apply(getPose());

				Rotation2d targetRotation =
						Rotation2d.fromDegrees(flippedPose.getRotation().getDegrees() - 0);

				Pose2d rotatedPose = new Pose2d(flippedPose.getTranslation(), targetRotation);

				return Optional.of(new Pose2d(rotatedPose.getTranslation(), rotatedPose.getRotation()));
			}

			return Optional.empty();
		}
	}

	public static Command driveToClosestPose(
			Drive drive, BooleanSupplier isRed, CommandXboxController driverController) {
		// Constants for feedforward scaling
		final double FF_MIN_RADIUS = 0.2; // meters
		final double FF_MAX_RADIUS = 0.6; // meters

		// Position and velocity tolerances
		final double TRANSLATION_TOLERANCE = 0.041; // meters
		final double HEADING_TOLERANCE = 1.5; // degrees
		final double VELOCITY_TOLERANCE = 0.075; // m/s

		// Create controllers
		ProfiledPIDController translationController =
				new ProfiledPIDController(
						6.25,
						0.0,
						0.0,
						new TrapezoidProfile.Constraints(
								3.0, 2.5) // MAX_TRANSLATION_SPEED, MAX_TRANSLATION_ACCEL
						);
		ExpDecayFF headingController = new ExpDecayFF(6.5, 1.0, HEADING_TOLERANCE);

		// Set tolerances
		translationController.setTolerance(TRANSLATION_TOLERANCE, VELOCITY_TOLERANCE);

		PoseAllignment poseAlignment = new PoseAllignment();

		return Commands.run(
						() -> {
							// Find the closest pose
							Pose2d currentPose = drive.getPose();
							Pose2d targetPose = findClosestPose(currentPose, isRed.getAsBoolean(), poseAlignment);

							// Calculate distance to target
							double currentDistance =
									currentPose.getTranslation().getDistance(targetPose.getTranslation());

							// Scale feedforward based on distance
							double ffScaler =
									MathUtil.clamp(
											(currentDistance - FF_MIN_RADIUS) / (FF_MAX_RADIUS - FF_MIN_RADIUS),
											0.0,
											1.0);

							// Calculate translation control output
							double driveVelocityScalar =
									translationController.getSetpoint().velocity * ffScaler
											+ translationController.calculate(currentDistance, 0.0);

							// If close enough to target, stop translation
							if (currentDistance < translationController.getPositionTolerance()) {
								driveVelocityScalar = 0.0;
							}

							// Calculate heading control output
							double headingError =
									getShortestAngleDifference(
											currentPose.getRotation().getDegrees(),
											targetPose.getRotation().getDegrees());

							double headingVelocity =
									headingController.calculate(
											currentPose.getRotation().getDegrees(),
											targetPose.getRotation().getDegrees());

							// If close enough to target heading, stop rotation
							if (Math.abs(headingError) < HEADING_TOLERANCE) {
								headingVelocity = 0.0;
							}

							// Calculate direction vector to target
							Rotation2d directionToTarget =
									targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle();

							// Convert velocity scalar to vector in direction of target
							Translation2d driveVelocity =
									new Translation2d(
											driveVelocityScalar * directionToTarget.getCos(),
											driveVelocityScalar * directionToTarget.getSin());

							// Create field-relative speeds
							ChassisSpeeds speeds =
									ChassisSpeeds.fromFieldRelativeSpeeds(
											-driveVelocity.getX(),
											-driveVelocity.getY(),
											headingVelocity,
											drive.getRotation());

							// Command the drive
							drive.runVelocity(speeds);

							// Logging
							Logger.recordOutput("VectorDrive/TargetPose", targetPose);
							Logger.recordOutput("VectorDrive/CurrentDistance", currentDistance);
							Logger.recordOutput("VectorDrive/TranslationOutput", driveVelocityScalar);
							Logger.recordOutput("VectorDrive/RotationOutput", headingVelocity);
							Logger.recordOutput("VectorDrive/FFScaler", ffScaler);
							Logger.recordOutput("VectorDrive/HeadingError", headingError);
							Logger.recordOutput(
									"VectorDrive/AtTranslationTarget", currentDistance < TRANSLATION_TOLERANCE);
							Logger.recordOutput(
									"VectorDrive/AtHeadingTarget", Math.abs(headingError) < HEADING_TOLERANCE);
							Logger.recordOutput("VectorDrive/DirectionToTarget", directionToTarget.getDegrees());
						},
						drive)
				.until(
						() -> {
							Pose2d currentPose = drive.getPose();
							Pose2d targetPose = findClosestPose(currentPose, isRed.getAsBoolean(), poseAlignment);

							double currentDistance =
									currentPose.getTranslation().getDistance(targetPose.getTranslation());
							double headingError =
									getShortestAngleDifference(
											currentPose.getRotation().getDegrees(),
											targetPose.getRotation().getDegrees());

							boolean atTranslationTarget =
									currentDistance < TRANSLATION_TOLERANCE
											&& Math.abs(translationController.getSetpoint().velocity)
													< VELOCITY_TOLERANCE;

							boolean atHeadingTarget = Math.abs(headingError) < HEADING_TOLERANCE;

							return (atTranslationTarget && atHeadingTarget)
									|| driverController.button(2).getAsBoolean();
						});
	}

	public static Command driveToClosestPosePathPlanner(Drive drive, BooleanSupplier isRed) {
		Pose2d targetPose =
				findClosestPose(drive.getPose(), isRed.getAsBoolean(), new PoseAllignment());
		return AutoBuilder.pathfindToPose(targetPose, PathConstraints.unlimitedConstraints(12.0));
	}

	private static Pose2d findClosestPose(
			Pose2d currentPose, boolean isRedAlliance, PoseAllignment poseAlignment) {
		List<Pose2d> allPoses = new ArrayList<>();

		if (isRedAlliance) {
			allPoses.addAll(poseAlignment.redLeft);
			allPoses.addAll(poseAlignment.redRight);
			allPoses.addAll(poseAlignment.HPRed);
		} else {
			allPoses.addAll(poseAlignment.blueLeft);
			allPoses.addAll(poseAlignment.blueRight);
			allPoses.addAll(poseAlignment.HPBlue);
		}

		return allPoses.stream()
				.min(
						Comparator.comparingDouble(
								pose -> currentPose.getTranslation().getDistance(pose.getTranslation())))
				.orElse(currentPose);
	}
}
