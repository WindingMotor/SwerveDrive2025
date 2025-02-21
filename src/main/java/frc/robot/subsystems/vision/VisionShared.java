// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.CameraConstants.Camera;
import frc.robot.subsystems.vision.IO_VisionBase.VisionInputs;
import frc.robot.subsystems.vision.IO_VisionReal.EstimateType;
import java.util.List;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Shared utility methods for vision processing across real and simulated implementations. */
public class VisionShared {
	// Constants for pose estimation tuning
	public static final double MAX_SINGLE_TAG_DISTANCE =
			4.0; // Maximum reliable distance for single tag detection
	public static final double DISTANCE_SCALING_FACTOR =
			30.0; // Scaling factor for standard deviation based on distance
	public static final double DISTANCE_DECAY_RATE = 0.35;
	public static final double MIN_STD_DEV = 0.15;
	public static final double MAX_STD_DEV = 3.0;

	/**
	 * Bundles all camera-related data to reduce map lookups and improve organization. This internal
	 * data structure keeps all related components together for better performance and cleaner code
	 * organization.
	 */
	public static class CameraData {
		final PhotonCamera camera;
		final PhotonCameraSim cameraSim;
		final PhotonPoseEstimator estimator;
		Matrix<N3, N1> stdDevMatrix;
		Pose3d[] visiblePoseBuffer; // Pre-allocated buffer for pose calculations
		PhotonPipelineResult latestResult;

		// Constructor for simulated vision with camera simulator
		public CameraData(
				PhotonCamera camera,
				PhotonCameraSim cameraSim,
				PhotonPoseEstimator estimator,
				Matrix<N3, N1> stdDevMatrix) {
			this.camera = camera;
			this.cameraSim = cameraSim;
			this.estimator = estimator;
			this.stdDevMatrix = stdDevMatrix;
			this.visiblePoseBuffer = new Pose3d[20]; // Buffer size based on max expected visible tags
			this.latestResult = new PhotonPipelineResult();
		}

		// Constructor for real vision without camera simulator
		public CameraData(
				PhotonCamera camera, PhotonPoseEstimator estimator, Matrix<N3, N1> stdDevMatrix) {
			this.camera = camera;
			this.cameraSim = null;
			this.estimator = estimator;
			this.stdDevMatrix = stdDevMatrix;
			this.visiblePoseBuffer = new Pose3d[20]; // Buffer size based on max expected visible tags
			this.latestResult = new PhotonPipelineResult();
		}
	}

	// Add this record before the SUB_Vision class
	public record CameraEstimationData(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevMatrix) {}

	/** Updates the target detection status for a specific camera. */
	public static void setHasTarget(VisionInputs inputs, Camera camera, boolean hasTarget) {
		switch (camera) {
			case FRONT_LEFT -> inputs.flHasTarget = hasTarget;
			case FRONT_RIGHT -> inputs.frHasTarget = hasTarget;
			case BACK_LEFT -> inputs.blHasTarget = hasTarget;
			case ELEVATED -> inputs.elHasTarget = hasTarget;
		}
	}

	/**
	 * Updates target information including visible tag poses and timestamps. Uses pre-allocated
	 * buffers to minimize garbage collection.
	 */
	public static void updateTargetInfo(
			VisionInputs inputs,
			Camera camera,
			PhotonPipelineResult result,
			CameraData data,
			AprilTagFieldLayout tagLayout,
			Pose2d robotPose) {

		// If no targets, reset the relevant camera inputs
		if (!result.hasTargets()) {
			switch (camera) {
				case FRONT_LEFT -> {
					inputs.flBestTargetID = -1;
					inputs.flVisibleTagPoses = new Pose3d[0];
					inputs.flTimestamp = 0.0;
				}
				case FRONT_RIGHT -> {
					inputs.frBestTargetID = -1;
					inputs.frVisibleTagPoses = new Pose3d[0];
					inputs.frTimestamp = 0.0;
				}
				case BACK_LEFT -> {
					inputs.blBestTargetID = -1;
					inputs.blVisibleTagPoses = new Pose3d[0];
					inputs.blTimestamp = 0.0;
				}
				case ELEVATED -> {
					inputs.elBestTargetID = -1;
					inputs.elVisibleTagPoses = new Pose3d[0];
					inputs.elTimestamp = 0.0;
				}
			}
			return;
		}

		// Safely get best target, defaulting to null if no targets
		PhotonTrackedTarget bestTarget = result.getBestTarget();
		List<PhotonTrackedTarget> targets = result.getTargets();

		// Fill pre-allocated buffer with visible tag poses
		int poseCount = 0;
		Transform3d robotToCameraTransform = data.estimator.getRobotToCameraTransform();
		Pose3d cameraPose = new Pose3d(robotPose).transformBy(robotToCameraTransform);
		for (PhotonTrackedTarget target : targets) {
			Optional<Pose3d> tagPose = tagLayout.getTagPose(target.getFiducialId());
			if (tagPose.isPresent() && poseCount < data.visiblePoseBuffer.length - 1) {
				data.visiblePoseBuffer[poseCount++] = tagPose.get();
				data.visiblePoseBuffer[poseCount++] = cameraPose;
			}
		}

		// Create final array of exact size needed
		Pose3d[] visiblePoses = new Pose3d[poseCount];
		System.arraycopy(data.visiblePoseBuffer, 0, visiblePoses, 0, poseCount);

		// Update inputs based on camera type
		switch (camera) {
			case FRONT_LEFT -> {
				inputs.flBestTargetID = bestTarget != null ? bestTarget.getFiducialId() : -1;
				inputs.flVisibleTagPoses = visiblePoses;
				inputs.flTimestamp = result.getTimestampSeconds();
			}
			case FRONT_RIGHT -> {
				inputs.frBestTargetID = bestTarget != null ? bestTarget.getFiducialId() : -1;
				inputs.frVisibleTagPoses = visiblePoses;
				inputs.frTimestamp = result.getTimestampSeconds();
			}
			case BACK_LEFT -> {
				inputs.blBestTargetID = bestTarget != null ? bestTarget.getFiducialId() : -1;
				inputs.blVisibleTagPoses = visiblePoses;
				inputs.blTimestamp = result.getTimestampSeconds();
			}
			case ELEVATED -> {
				inputs.elBestTargetID = bestTarget != null ? bestTarget.getFiducialId() : -1;
				inputs.elVisibleTagPoses = visiblePoses;
				inputs.elTimestamp = result.getTimestampSeconds();
			}
		}
	}

	/** Sets the estimated pose for a specific camera. */
	public static void setPoseEstimate(VisionInputs inputs, Camera camera, Pose2d pose) {
		switch (camera) {
			case FRONT_LEFT -> inputs.flEstimatedPose = pose;
			case FRONT_RIGHT -> inputs.frEstimatedPose = pose;
			case BACK_LEFT -> inputs.blEstimatedPose = pose;
			case ELEVATED -> inputs.elEstimatedPose = pose;
		}
	}

	public static void setEstimateType(
			VisionInputs inputs, Camera camera, EstimateType estimateType) {
		switch (camera) {
			case FRONT_LEFT -> inputs.flEstimateType = estimateType.toString();
			case FRONT_RIGHT -> inputs.frEstimateType = estimateType.toString();
			case BACK_LEFT -> inputs.blEstimateType = estimateType.toString();
			case ELEVATED -> inputs.elEstimateType = estimateType.toString();
		}
	}

	public static void updateClosestTargetInfo(
			VisionInputs inputs,
			Camera camera,
			PhotonPipelineResult result,
			Pose2d robotPose,
			AprilTagFieldLayout tagLayout) {

		if (!result.hasTargets()) {
			return;
		}

		List<PhotonTrackedTarget> targets = result.getTargets();
		double minDistance = Double.MAX_VALUE;
		int closestTargetId = -1;

		for (PhotonTrackedTarget target : targets) {
			Optional<Pose3d> tagPose = tagLayout.getTagPose(target.getFiducialId());
			if (tagPose.isPresent()) {
				double distance =
						tagPose.get().toPose2d().getTranslation().getDistance(robotPose.getTranslation());
				if (distance < minDistance) {
					minDistance = distance;
					closestTargetId = target.getFiducialId();
				}
			}
		}
	}

	/**
	 * Calculates pose estimation uncertainty using a physics-informed statistical model that balances
	 * tag observation quality and quantity. The model behaves differently based on single/multi-tag
	 * scenarios:
	 *
	 * <p><b>Single Tag Mode:</b>
	 *
	 * <ul>
	 *   <li>Uncertainty grows exponentially as the robot moves further from the tag
	 *   <li>Considers PhotonVision's pose ambiguity score (0=confident, 1=ambiguous)
	 *   <li>Ignores single tags beyond {@value #MAX_SINGLE_TAG_DISTANCE} meters
	 *   <li>Example: At 2 meters with ambiguity 0.3 → base uncertainty × 2.7
	 * </ul>
	 *
	 * <p><b>Multi-Tag Mode:</b>
	 *
	 * <ul>
	 *   <li>Rewards additional tags with diminishing returns (logarithmic scaling)
	 *   <li>2 tags → 85% of base multi-tag uncertainty
	 *   <li>4 tags → 70% of base multi-tag uncertainty
	 *   <li>Prevents overconfidence from too many nearby tags
	 * </ul>
	 *
	 * <p><b>Safety Limits:</b>
	 *
	 * <ul>
	 *   <li>Never less than {@value #MIN_STD_DEV}m (sensor resolution limit)
	 *   <li>Never more than {@value #MAX_STD_DEV}m (practical tracking cutoff)
	 * </ul>
	 */
	public static void updateEstimationStdDevs(
			Camera camera,
			EstimatedRobotPose poseResult,
			List<PhotonTrackedTarget> targets,
			CameraData data,
			AprilTagFieldLayout tagLayout) {

		int numTags = 0;
		double totalDistance = 0.0;
		Pose2d estimatedPose2d = poseResult.estimatedPose.toPose2d();

		// Calculate weighted average distance with ambiguity
		for (PhotonTrackedTarget target : targets) {
			Optional<Pose3d> tagPose = tagLayout.getTagPose(target.getFiducialId());
			if (tagPose.isPresent()) {
				numTags++;
				double distance =
						tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose2d.getTranslation());
				double ambiguityWeight = 1.0 + target.getPoseAmbiguity();
				totalDistance += distance * ambiguityWeight;
			}
		}

		if (numTags == 0) {
			data.stdDevMatrix = camera.singleTagStdDevs;
			return;
		}

		double avgDist = totalDistance / numTags;

		if (numTags > 1) {
			double multiTagWeight = 1.0 / (1.0 + Math.log1p(numTags));
			data.stdDevMatrix = camera.multiTagStdDevs.times(multiTagWeight);
		} else {
			if (avgDist > MAX_SINGLE_TAG_DISTANCE) {
				data.stdDevMatrix = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
			} else {
				double distanceFactor = Math.exp(avgDist * DISTANCE_DECAY_RATE);
				Matrix<N3, N1> calculatedStdDev = camera.singleTagStdDevs.times(distanceFactor);

				// Create new matrix with clipped values
				SimpleMatrix clippedMatrix = new SimpleMatrix(3, 1);
				for (int i = 0; i < 3; i++) {
					double value = calculatedStdDev.get(i, 0);
					clippedMatrix.set(i, 0, Math.min(MAX_STD_DEV, Math.max(MIN_STD_DEV, value)));
				}
				data.stdDevMatrix = new Matrix<>(clippedMatrix);
			}
		}
	}
}
