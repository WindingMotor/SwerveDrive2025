// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.LocalADStarAK;
import frc.robot.commands.drive.DriveCommands.ZonePose;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.RobotMode;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.apriltag.SUB_Apriltag;
import frc.robot.subsystems.apriltag.SUB_Apriltag.VisionEstimates;
import frc.robot.subsystems.apriltag.SUB_Apriltag.VisionMeasurement;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

	private final AprilTagFieldLayout aprilTagFieldLayout =
			AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

	// TunerConstants doesn't include these constants, so they are declared locally
	static final double ODOMETRY_FREQUENCY =
			new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
	public static final double DRIVE_BASE_RADIUS =
			Math.max(
					Math.max(
							Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
							Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
					Math.max(
							Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
							Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

	// PathPlanner config constants
	private static final double ROBOT_MASS_KG = 59.0;
	private static final double ROBOT_MOI = 6.883;
	private static final double WHEEL_COF = 1.2;
	private static final RobotConfig PP_CONFIG =
			new RobotConfig(
					ROBOT_MASS_KG,
					ROBOT_MOI,
					new ModuleConfig(
							TunerConstants.FrontLeft.WheelRadius,
							TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
							WHEEL_COF,
							DCMotor.getKrakenX60Foc(1)
									.withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
							TunerConstants.FrontLeft.SlipCurrent,
							1),
					getModuleTranslations());

	static final Lock odometryLock = new ReentrantLock();
	private final IO_GyroBase gyroIO;
	private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
	private final Module[] modules = new Module[4]; // FL, FR, BL, BR
	private final SysIdRoutine sysId;
	private final Alert gyroDisconnectedAlert =
			new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
	private Rotation2d rawGyroRotation = new Rotation2d();
	private SwerveModulePosition[] lastModulePositions = // For delta tracking
			new SwerveModulePosition[] {
				new SwerveModulePosition(),
				new SwerveModulePosition(),
				new SwerveModulePosition(),
				new SwerveModulePosition()
			};
	private SwerveDrivePoseEstimator poseEstimator =
			new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

	// private SUB_Vision vision;

	private Pair<Integer, Double> closestTagData = Pair.of(-1, Double.MAX_VALUE);

	SUB_Apriltag apriltag;
	private final List<Pose2d> estimates = new ArrayList<>();
	private final List<Pose3d> targets = new ArrayList<>();

	public Drive(
			IO_GyroBase gyroIO,
			//	SUB_Vision vision,
			IO_ModuleBase flModuleIO,
			IO_ModuleBase frModuleIO,
			IO_ModuleBase blModuleIO,
			IO_ModuleBase brModuleIO) {
		this.gyroIO = gyroIO;
		//	this.vision = vision;

		apriltag = new SUB_Apriltag();

		modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
		modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
		modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
		modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

		// Usage reporting for swerve template
		HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

		// Start odometry thread
		PhoenixOdometryThread.getInstance().start();

		// Configure AutoBuilder for PathPlanner
		AutoBuilder.configure(
				this::getPose,
				this::setPose,
				this::getChassisSpeeds,
				this::runVelocity,
				new PPHolonomicDriveController(
						new PIDConstants(4.45, 0.0, 0.0), new PIDConstants(5.3, 0.0, 0.0)),
				PP_CONFIG,
				() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
				this);
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback(
				(activePath) -> {
					Logger.recordOutput(
							"Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
				});
		PathPlannerLogging.setLogTargetPoseCallback(
				(targetPose) -> {
					Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
				});

		// Configure SysId
		sysId =
				new SysIdRoutine(
						new SysIdRoutine.Config(
								null,
								null,
								null,
								(state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
						new SysIdRoutine.Mechanism(
								(voltage) -> runCharacterization(voltage.in(Volts)), null, this));

		// Get alliance
	}

	@Override
	public void periodic() {

		odometryLock.lock(); // Prevents odometry updates while reading data
		gyroIO.updateInputs(gyroInputs);

		Logger.processInputs("Drive/Gyro", gyroInputs);
		for (var module : modules) {
			module.periodic();
		}
		odometryLock.unlock();

		// Stop moving when disabled
		if (DriverStation.isDisabled()) {
			for (var module : modules) {
				module.stop();
			}
		}

		// Log empty setpoint states when disabled
		if (DriverStation.isDisabled()) {
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
			Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
		}

		// Update odometry
		double[] sampleTimestamps =
				modules[0].getOdometryTimestamps(); // All signals are sampled together
		int sampleCount = sampleTimestamps.length;
		for (int i = 0; i < sampleCount; i++) {
			// Read wheel positions and deltas from each module
			SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
			SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
			for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
				modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
				moduleDeltas[moduleIndex] =
						new SwerveModulePosition(
								modulePositions[moduleIndex].distanceMeters
										- lastModulePositions[moduleIndex].distanceMeters,
								modulePositions[moduleIndex].angle);
				lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
			}

			// Update gyro angle
			if (gyroInputs.connected) {
				// Use the real gyro angle
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else {
				// Use the angle delta from the kinematics and module deltas
				Twist2d twist = kinematics.toTwist2d(moduleDeltas);
				rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
			}

			// Apply update
			poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
		}

		// Update gyro alert
		gyroDisconnectedAlert.set(!gyroInputs.connected && RobotConstants.ROBOT_MODE != RobotMode.SIM);

		List<SUB_Apriltag.TimestampedYaw> yawMeasurements = new ArrayList<>();

		yawMeasurements.add(new SUB_Apriltag.TimestampedYaw(rawGyroRotation, Timer.getFPGATimestamp()));

		apriltag.addYawMeasurements(yawMeasurements);
		apriltag.updateLastRobotPose(getPose());

		VisionEstimates visionEstimates = apriltag.getUnreadResults();

		for (VisionMeasurement measurement : visionEstimates.measurements()) {
			if (measurement.stdDevs() == null) {
				poseEstimator.addVisionMeasurement(measurement.visionPose(), measurement.timestamp());
			} else {
				poseEstimator.addVisionMeasurement(
						measurement.visionPose(), measurement.timestamp(), measurement.stdDevs());
			}
		}

		estimates.clear();
		estimates.addAll(visionEstimates.getPoses());

		targets.clear();
		targets.addAll(visionEstimates.targets());

		// Update camera positions
		updateCameraPositions();

		if (!visionEstimates.measurements().isEmpty()) {
			// Log all vision measurements
			Pose2d[] visionPoses =
					visionEstimates.measurements().stream().map(m -> m.visionPose()).toArray(Pose2d[]::new);

			Logger.recordOutput("Vision/EstimatedPosesAverage", visionPoses);

			// Also log the most recent vision measurement
			Logger.recordOutput("Vision/LatestEstimate", visionPoses[visionPoses.length - 1]);
		}

		// Don't forget to update closestTagData
		closestTagData = getClosestAprilTagID();
	}

	private void updateCameraPositions() {

		// Convert Pose2d to Pose3d for camera position transformation
		Pose3d robotPose3d = new Pose3d(getPose());

		// Transform camera positions to global coordinate system
		Pose3d[] globalCameraPositions = new Pose3d[CameraConstants.CAMERA_POSITIONS.length];
		for (int i = 0; i < CameraConstants.CAMERA_POSITIONS.length; i++) {
			globalCameraPositions[i] =
					robotPose3d.transformBy(
							new Transform3d(
									CameraConstants.CAMERA_POSITIONS[i].getTranslation(),
									CameraConstants.CAMERA_POSITIONS[i].getRotation()));
		}

		// Record camera positions for visualization
		Logger.recordOutput("CameraPositions", globalCameraPositions);
		Logger.recordOutput("Alliance Zone Poses", getAllZonePoses());
	}

	/*
	 * Returns all the zone poses for the current alliance with flipping.
	 */
	public static Pose2d[] getAllZonePoses() {
		List<Pose2d> poses = new ArrayList<>();

		for (ZonePose zonePose : ZonePose.values()) {
			Optional<Pose2d> poseForAlliance = zonePose.getPoseForAlliance();
			if (poseForAlliance.isPresent()) {
				poses.add(poseForAlliance.get());
			}
		}

		return poses.toArray(new Pose2d[0]);
	}

	/**
	 * Runs the drive at the desired velocity.
	 *
	 * @param speeds Speeds in meters/sec
	 */
	public void runVelocity(ChassisSpeeds speeds) {
		// Calculate module setpoints
		ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
		SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

		// Log unoptimized setpoints and setpoint speeds
		Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
		Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

		// Send setpoints to modules
		for (int i = 0; i < 4; i++) {
			modules[i].runSetpoint(setpointStates[i]);
		}

		// Log optimized setpoints (runSetpoint mutates each state)
		Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
	}

	/** Runs the drive in a straight line with the specified drive output. */
	public void runCharacterization(double output) {
		for (int i = 0; i < 4; i++) {
			modules[i].runCharacterization(output);
		}
	}

	/** Stops the drive. */
	public void stop() {
		runVelocity(new ChassisSpeeds());
	}

	/**
	 * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
	 * return to their normal orientations the next time a nonzero velocity is requested.
	 */
	public void stopWithX() {
		Rotation2d[] headings = new Rotation2d[4];
		for (int i = 0; i < 4; i++) {
			headings[i] = getModuleTranslations()[i].getAngle();
		}
		kinematics.resetHeadings(headings);
		stop();
	}

	/** Returns a command to run a quasistatic test in the specified direction. */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return run(() -> runCharacterization(0.0))
				.withTimeout(1.0)
				.andThen(sysId.quasistatic(direction));
	}

	/** Returns a command to run a dynamic test in the specified direction. */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
	}

	/** Returns the module states (turn angles and drive velocities) for all of the modules. */
	@AutoLogOutput(key = "SwerveStates/Measured")
	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}

	/** Returns the module positions (turn angles and drive positions) for all of the modules. */
	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] states = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getPosition();
		}
		return states;
	}

	/** Returns the measured chassis speeds of the robot. */
	@AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
	private ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	/** Returns the position of each module in radians. */
	public double[] getWheelRadiusCharacterizationPositions() {
		double[] values = new double[4];
		for (int i = 0; i < 4; i++) {
			values[i] = modules[i].getWheelRadiusCharacterizationPosition();
		}
		return values;
	}

	/** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
	public double getFFCharacterizationVelocity() {
		double output = 0.0;
		for (int i = 0; i < 4; i++) {
			output += modules[i].getFFCharacterizationVelocity() / 4.0;
		}
		return output;
	}

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Odometry/Robot")
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/** Returns the current odometry rotation. */
	public Rotation2d getRotation() {
		return getPose().getRotation();
	}

	/** Resets the current odometry pose. */
	public void setPose(Pose2d pose) {
		poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
	}

	/** Adds a new timestamped vision measurement. */
	public void addVisionMeasurement(
			Pose2d visionRobotPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs) {
		poseEstimator.addVisionMeasurement(
				visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
	}

	/** Returns the maximum linear speed in meters per sec. */
	public double getMaxLinearSpeedMetersPerSec() {
		return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
	}

	/** Returns the maximum angular speed in radians per sec. */
	public double getMaxAngularSpeedRadPerSec() {
		return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
	}

	/** Returns an array of module translations. */
	public static Translation2d[] getModuleTranslations() {
		return new Translation2d[] {
			new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
			new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
			new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
			new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
		};
	}

	/**
	 * Get the ID of the closest AprilTag.
	 *
	 * @return The ID of the closest AprilTag
	 */
	public Pair<Integer, Double> getClosestAprilTagID() {
		Pose2d currentPose = getPose();
		double minDistance = Double.MAX_VALUE;
		int closestTagId = -1;

		// Iterate through all AprilTags (1-22)
		for (int tagId = 1; tagId <= 22; tagId++) {
			Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tagId);
			if (tagPose.isPresent()) {
				double distance =
						currentPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());

				if (distance < minDistance) {
					minDistance = distance;
					closestTagId = tagId;
				}
			}
		}

		Logger.recordOutput("ClosestAprilTag/Id", closestTagId);
		Logger.recordOutput("ClosestAprilTag/Distance", minDistance);

		return Pair.of(closestTagId, minDistance);
	}

	/**
	 * Get the ID of the closest AprilTag.
	 *
	 * @return The ID of the closest AprilTag
	 */
	public Translation2d getApriltagLocation(int id) {
		if (id > 0) {
			Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(id);
			return new Translation2d(pose3d.get().getX(), pose3d.get().getY());
		} else {
			return new Translation2d();
		}
	}

	public Pair<Integer, Double> getRecentClosestTagData() {
		return closestTagData;
	}
}
