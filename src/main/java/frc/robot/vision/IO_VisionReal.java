// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.CameraConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Real implementation of the vision system using PhotonVision and AprilTags This class handles
 * multiple cameras to estimate robot position on the field
 */
public class IO_VisionReal implements IO_VisionBase {

	private final AprilTagFieldLayout fieldLayout;
	private final Map<CameraConstants.Camera, PhotonCamera> cameras = new HashMap<>();
	private final Map<CameraConstants.Camera, PhotonPoseEstimator> poseEstimators = new HashMap<>();
	private Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();

	// Current results for each camera, updated in updateInputs
	private final Map<CameraConstants.Camera, PhotonPipelineResult> currentResults = new HashMap<>();
	private final Map<CameraConstants.Camera, Matrix<N3, N1>> currentStdDevs = new HashMap<>();

	private Pose3d lastCurrentPose = new Pose3d();

	/** Constructor initializes all cameras and their pose estimators */
	public IO_VisionReal() {
		fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

		for (CameraConstants.Camera cam : CameraConstants.Camera.values()) {

			cameras.put(cam, new PhotonCamera(cam.name));
			currentResults.put(cam, new PhotonPipelineResult()); // Empty initial result
			currentStdDevs.put(cam, cam.singleTagStdDevs);

			Transform3d robotToCam = new Transform3d(cam.translation, cam.rotation);
			PhotonPoseEstimator estimator =
					new PhotonPoseEstimator(
							fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

			estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
			poseEstimators.put(cam, estimator);
		}
	}

	/** Updates vision inputs with the latest target information from each camera */
	@Override
	public void updateInputs(VisionInputs inputs) {
		List<Pose3d> leftTagPoses = new ArrayList<>();
		List<Pose3d> frontLeftTopPoses = new ArrayList<>();
		List<Pose3d> backLeftTagPoses = new ArrayList<>();

		// Update all camera results first
		for (Map.Entry<CameraConstants.Camera, PhotonCamera> entry : cameras.entrySet()) {
			CameraConstants.Camera cam = entry.getKey();
			PhotonCamera camera = entry.getValue();

			List<PhotonPipelineResult> results = camera.getAllUnreadResults();
			if (!results.isEmpty()) {
				// Get the most recent result
				results.sort((a, b) -> Double.compare(b.getTimestampSeconds(), a.getTimestampSeconds()));
				currentResults.put(cam, results.get(0));
			}
		}

		// Process the results for each camera
		for (Map.Entry<CameraConstants.Camera, PhotonCamera> entry : cameras.entrySet()) {
			CameraConstants.Camera cam = entry.getKey();
			PhotonPipelineResult result = currentResults.get(cam);
			if (result.hasTargets()) {
				PhotonTrackedTarget bestTarget = result.getBestTarget();

				// Calculate camera position based on robot pose
				Pose3d cameraPose =
						lastCurrentPose.transformBy(new Transform3d(cam.translation, cam.rotation));

				// Process targets for each camera
				List<Pose3d> currentCameraPoses = new ArrayList<>();
				for (PhotonTrackedTarget target : result.getTargets()) {
					Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
					if (tagPose.isPresent()) {
						// Add both AprilTag pose and camera position pose
						currentCameraPoses.add(tagPose.get());
						currentCameraPoses.add(cameraPose);
					}
				}

				switch (cam) {
					case LEFT_CAM:
						inputs.hasLeftTarget = true;
						inputs.leftBestTargetID = bestTarget.getFiducialId();
						break;
					case FRONT_LEFT_TOP_CAM:
						inputs.hasFrontLeftTopTarget = true;
						inputs.frontLeftTopBestTargetID = bestTarget.getFiducialId();
						break;
					case BACK_LEFT_CAM:
						inputs.hasBackLeftTarget = true;
						inputs.backLeftBestTargetID = bestTarget.getFiducialId();
						break;
				}
			}
		}

		// Update inputs with visible tag poses
		inputs.leftVisibleTagPoses = leftTagPoses.toArray(new Pose3d[0]);
		inputs.rightVisibleTagPoses = frontLeftTopPoses.toArray(new Pose3d[0]);
		inputs.backLeftVisibleTagPoses = backLeftTagPoses.toArray(new Pose3d[0]);
		inputs.lastEstimatedPose =
				lastEstimatedPose.isPresent() ? lastEstimatedPose.get().estimatedPose : null;
	}

	/** Updates robot pose estimation using data from all cameras */
	public void updatePoseEstimation(Pose2d currentPose) {
		lastCurrentPose = new Pose3d(currentPose);
		Map<CameraConstants.Camera, EstimatedRobotPose> cameraEstimates = new HashMap<>();

		for (Map.Entry<CameraConstants.Camera, PhotonPoseEstimator> entry : poseEstimators.entrySet()) {

			CameraConstants.Camera cam = entry.getKey();
			PhotonPoseEstimator estimator = entry.getValue();
			PhotonPipelineResult result = currentResults.get(cam);

			if (!result.hasTargets()) {
				continue;
			}

			// Filter out unreliable single-tag measurements
			if (result.getTargets().size() == 1) {
				PhotonTrackedTarget target = result.getBestTarget();
				if (target.getPoseAmbiguity() > CameraConstants.MAXIMUM_AMBIGUITY) {
					Logger.recordOutput(
							"Vision/" + cam.name + "/RejectedAmbiguity", target.getPoseAmbiguity());
					continue;
				}
			}

			// Set reference pose to current robot pose
			estimator.setReferencePose(currentPose);

			// Update estimator
			Optional<EstimatedRobotPose> poseResult = estimator.update(result);

			// If estimation is successful record output, update estimates, and update stdDevs
			if (poseResult.isPresent()) {
				EstimatedRobotPose estimate = poseResult.get();
				updateEstimationStdDevs(cam, poseResult, result.getTargets());
				cameraEstimates.put(cam, estimate);

				Logger.recordOutput("Vision/" + cam.name + "/EstimatedPose", estimate.estimatedPose);
				Logger.recordOutput("Vision/" + cam.name + "/TimestampSeconds", estimate.timestampSeconds);
				Logger.recordOutput("Vision/" + cam.name + "/TagCount", estimate.targetsUsed.size());
			}
		}

		if (!cameraEstimates.isEmpty()) {
			EstimatedRobotPose combinedPose = combineEstimates(cameraEstimates);
			lastEstimatedPose = Optional.of(combinedPose);
			Logger.recordOutput("Vision/CombinedEstimatedPose", combinedPose.estimatedPose);
		}
	}

	/**
	 * Updates the standard deviation for pose estimation based on camera results.
	 *
	 * <p>Calculates and adjusts standard deviation based on: - Number of tags used in estimation -
	 * Ambiguity of tag detections
	 *
	 * @param camera The camera being evaluated
	 * @param poseResult The estimated robot pose from the camera
	 * @param targets The AprilTag targets used in pose estimation
	 */
	private void updateEstimationStdDevs(
			CameraConstants.Camera camera,
			Optional<EstimatedRobotPose> poseResult,
			List<PhotonTrackedTarget> targets) {

		if (poseResult.isEmpty()) {
			currentStdDevs.put(camera, camera.singleTagStdDevs);
			return;
		}

		Matrix<N3, N1> estStdDevs = camera.singleTagStdDevs;
		int numTags = 0;
		double avgDist = 0;

		for (PhotonTrackedTarget target : targets) {
			Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
			if (tagPose.isEmpty()) {
				continue;
			}
			numTags++;
			avgDist +=
					tagPose
							.get()
							.toPose2d()
							.getTranslation()
							.getDistance(poseResult.get().estimatedPose.toPose2d().getTranslation());
		}

		if (numTags == 0) {
			currentStdDevs.put(camera, camera.singleTagStdDevs);
			return;
		}

		avgDist /= numTags;

		if (numTags > 1) {
			estStdDevs = camera.multiTagStdDevs;
		}

		if (numTags == 1 && avgDist > 4) {
			estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
		} else {
			estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
		}

		currentStdDevs.put(camera, estStdDevs);
	}

	/**
	 * Combines pose estimates from multiple cameras using weighted averaging.
	 *
	 * <p>Calculates a combined pose by: - Weighting each camera's estimate based on its standard
	 * deviation - Computing weighted averages for X, Y, and rotation
	 *
	 * @param estimates A map of camera estimates to be combined
	 * @return A combined EstimatedRobotPose representing the most accurate position
	 */
	private EstimatedRobotPose combineEstimates(
			Map<CameraConstants.Camera, EstimatedRobotPose> estimates) {
		double weightedX = 0;
		double weightedY = 0;
		double weightedRot = 0;
		double totalWeightX = 0;
		double totalWeightY = 0;
		double totalWeightRot = 0;

		EstimatedRobotPose firstEstimate = estimates.values().iterator().next();

		for (Map.Entry<CameraConstants.Camera, EstimatedRobotPose> entry : estimates.entrySet()) {
			CameraConstants.Camera cam = entry.getKey();
			EstimatedRobotPose estimate = entry.getValue();
			Matrix<N3, N1> stdDevs = currentStdDevs.get(cam);

			double weightX = 1.0 / (stdDevs.get(0, 0) * stdDevs.get(0, 0));
			double weightY = 1.0 / (stdDevs.get(1, 0) * stdDevs.get(1, 0));
			double weightRot = 1.0 / (stdDevs.get(2, 0) * stdDevs.get(2, 0));

			Pose3d pose = estimate.estimatedPose;
			weightedX += pose.getX() * weightX;
			weightedY += pose.getY() * weightY;
			weightedRot += pose.getRotation().getZ() * weightRot;

			totalWeightX += weightX;
			totalWeightY += weightY;
			totalWeightRot += weightRot;
		}

		double finalX = weightedX / totalWeightX;
		double finalY = weightedY / totalWeightY;
		double finalRot = weightedRot / totalWeightRot;

		Pose3d combinedPose =
				new Pose3d(new Translation3d(finalX, finalY, 0), new Rotation3d(0, 0, finalRot));

		return new EstimatedRobotPose(
				combinedPose,
				firstEstimate.timestampSeconds,
				firstEstimate.targetsUsed,
				firstEstimate.strategy);
	}

	@Override
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
		return lastEstimatedPose;
	}
}
