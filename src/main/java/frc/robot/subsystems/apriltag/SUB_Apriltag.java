// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.CameraConstants;
import frc.robot.subsystems.apriltag.SUB_Apriltag.TimestampedYaw;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Manages all of the robot's cameras. */
@Logged(strategy = Strategy.OPT_IN)
public final class SUB_Apriltag {

	/**
	 * Represents a measurement from vision to apply to the pose estimator.
	 *
	 * @see {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
	 */
	public static final record VisionMeasurement(
			Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
		/**
		 * Represents a measurement from vision to apply to the pose estimator.
		 *
		 * @see {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
		 */
		public VisionMeasurement(Pose2d visionPose, double timestamp) {
			this(visionPose, timestamp, null);
		}
	}

	/** Contains a yaw measurement alongside the timestamp of the measurement, in seconds. */
	public static final record TimestampedYaw(Rotation2d yaw, double timestamp) {}

	private static final AprilTagFields kField = AprilTagFields.k2025Reefscape;
	private static final PoseStrategy kStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
	private static final PoseStrategy kBackupStrategy = PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;

	private final AprilTagFieldLayout aprilTags;
	private final Camera[] cameras;

	private Pose2d lastRobotPose = new Pose2d();

	/*
		 *
		 * 		initializeCamera(Camera.FRONT_LEFT, "OV2311_4");
	initializeCamera(Camera.FRONT_RIGHT, "OV2311_5");
	initializeCamera(Camera.BACK_LEFT, "OV9281_03");
	initializeCamera(Camera.ELEVATED, "OV9281_02");

		 */
	public SUB_Apriltag() {
		aprilTags = AprilTagFieldLayout.loadField(kField);
		cameras =
				new Camera[] {
					new Camera("OV9281_02", CameraConstants.Camera.ELEVATED.getTransform()), // FRONT Camera
					// new Camera(
					//	"OV2311_5", CameraConstants.Camera.FRONT_RIGHT.getTransform()), //  RIGHT Camera
					new Camera("OV9281_03", CameraConstants.Camera.FRONT_LEFT.getTransform()), // LEFT Camera
				};

		NetworkTableInstance.getDefault()
				.getBooleanTopic("/photonvision/use_new_cscore_frametime")
				.publish()
				.set(true);
	}

	/**
	 * Adds yaw measurements to be used for pose estimation.
	 *
	 * @param yawMeasurements Robot yaw measurements since the last robot cycle.
	 */
	public void addYawMeasurements(List<TimestampedYaw> yawMeasurements) {
		for (var camera : cameras) {
			camera.addYawMeasurements(yawMeasurements);
		}
	}

	public void updateLastRobotPose(Pose2d newPose2d) {
		lastRobotPose = newPose2d;
	}

	/** Gets unread results from all cameras. */
	public VisionEstimates getUnreadResults() {
		List<VisionMeasurement> measurements = new ArrayList<>();
		List<Pose3d> targets = new ArrayList<>();

		for (var camera : cameras) {
			var result = camera.getUnreadResults();

			// Filter measurements to reduce jitter
			for (VisionMeasurement measurement : result.measurements()) {
				// Calculate distance between vision measurement and last known pose
				double poseDifference =
						lastRobotPose.getTranslation().getDistance(measurement.visionPose().getTranslation());

				// Only keep measurements within a reasonable threshold
				if (poseDifference < 1.0 || lastRobotPose.equals(new Pose2d())) {
					measurements.add(measurement);
				}
			}

			targets.addAll(result.targets());
		}

		return new VisionEstimates(measurements, targets);
	}

	private class Camera {

		private final PhotonCamera camera;
		private final PhotonPoseEstimator estimator;

		/**
		 * Create a camera.
		 *
		 * @param cameraName The configured name of the camera.
		 * @param robotToCamera The {@link Transform3d} from the robot's center to the camera.
		 */
		private Camera(String cameraName, Transform3d robotToCamera) {
			camera = new PhotonCamera(cameraName);
			estimator = new PhotonPoseEstimator(aprilTags, kStrategy, robotToCamera);
			estimator.setPrimaryStrategy(kStrategy);
			estimator.setMultiTagFallbackStrategy(kBackupStrategy);
		}

		/**
		 * Adds yaw measurements to be used for pose estimation.
		 *
		 * @param yawMeasurements Robot yaw measurements since the last robot cycle.
		 */
		private void addYawMeasurements(List<TimestampedYaw> yawMeasurements) {
			for (var measurement : yawMeasurements) {
				estimator.addHeadingData(measurement.timestamp(), measurement.yaw());
			}
		}

		/** Gets unread results from the camera. */
		private VisionEstimates getUnreadResults() {
			List<VisionMeasurement> measurements = new ArrayList<>();
			List<Pose3d> targets = new ArrayList<>();

			for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

				var estimate = estimator.update(result);
				if (estimate.isEmpty() || estimate.get().targetsUsed.isEmpty()) continue;

				var target = estimate.get().targetsUsed.get(0);
				int id = target.fiducialId;
				if (!useTag(id)) continue;

				var tagLocation = aprilTags.getTagPose(id);
				if (tagLocation.isEmpty()) continue;

				// Calculate the std
				double distance = target.bestCameraToTarget.getTranslation().getNorm();
				double std = 0.1 * Math.pow(distance, 2.0);

				measurements.add(
						new VisionMeasurement(
								estimate.get().estimatedPose.toPose2d(),
								estimate.get().timestampSeconds,
								VecBuilder.fill(std, std, 1000.0)));

				targets.add(tagLocation.get());
			}

			return new VisionEstimates(measurements, targets);
		}

		/**
		 * Returns {@code true} if an AprilTag should be utilized.
		 *
		 * @param id The ID of the AprilTag.
		 */
		private boolean useTag(int id) {
			return (DriverStation.getAlliance().get() == Alliance.Blue)
					? (id >= 17 && id <= 22)
					: (id >= 6 && id <= 11);
		}
	}

	public static final record VisionEstimates(
			List<VisionMeasurement> measurements, List<Pose3d> targets) {
		/** Returns all robot pose estimates in the calculated measurements. */
		public List<Pose2d> getPoses() {
			return measurements.stream().map(m -> m.visionPose()).toList();
		}
	}
}
