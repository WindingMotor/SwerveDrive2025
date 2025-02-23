// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.drive;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.math.ExpDecayFF;
import java.text.DecimalFormat;
import java.text.NumberFormat;
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

	private static final ExpDecayFF rotationController = new ExpDecayFF(6.0, 1.0, 0.25);

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
		ExpDecayFF xController = new ExpDecayFF(150.0, 1.5, 0.032);
		ExpDecayFF yController = new ExpDecayFF(150.0, 1.5, 0.032);
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

	public static Command driveToZone(Drive drive, ZonePose zonePose) {
		return driveToPose(drive, zonePose.getPose());
	}

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
	if (id == 1) {
		return DriveCommands.driveToPose(
				drive, new Pose2d(new Translation2d(16.94, 1.2), Rotation2d.fromDegrees(-55)));

	} else if (id == 2) {
		return DriveCommands.driveToPose(
				drive, new Pose2d(new Translation2d(16.31, 7.35), Rotation2d.fromDegrees(44)));
	} else {
		return new PrintCommand("Nothing...");
	}
		*/

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

	public enum ZonePose {
		NONE(new Translation2d(), ZoneAngle.NONE),
		FORWARD(new Translation2d(), ZoneAngle.FORWARD),
		BACKWARD(new Translation2d(), ZoneAngle.BACKWARD),
		RIGHT(new Translation2d(), ZoneAngle.RIGHT),
		LEFT(new Translation2d(), ZoneAngle.LEFT),

		// SOURCE
		SOURCE_RIGHT(new Translation2d(16.25, 7.25), ZoneAngle.SOURCE_RIGHT),
		SOURCE_LEFT(new Translation2d(16.8, 0.95), ZoneAngle.SOURCE_LEFT),

		// BOTTOM RIGHT
		REEF_BOTTOM_RIGHT_TOP(new Translation2d(13.62, 5.16), ZoneAngle.REEF_BOTTOM_RIGHT),
		REEF_BOTTOM_RIGHT_BOTTOM(new Translation2d(13.89, 5.1), ZoneAngle.REEF_BOTTOM_RIGHT),

		// BOTTOM LEFT
		// REEF_BOTTOM_LEFT(new Translation2d(), ZoneAngle.REEF_BOTTOM_LEFT),

		// BOTTOM
		REEF_BOTTOM_LEFT(new Translation2d(14.384, 3.852), ZoneAngle.REEF_BOTTOM),
		REEF_BOTTOM_RIGHT(new Translation2d(14.384, 4.181), ZoneAngle.REEF_BOTTOM),

		// TOP RIGHT
		REEF_TOP_RIGHT_BOTTOM(new Translation2d(12.553, 5.249), ZoneAngle.REEF_TOP_RIGHT),
		REEF_TOP_RIGHT_TOP(new Translation2d(12.265, 5.083), ZoneAngle.REEF_TOP_RIGHT),

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
}
