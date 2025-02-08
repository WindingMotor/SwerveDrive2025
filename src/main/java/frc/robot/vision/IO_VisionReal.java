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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.CameraConstants.Camera;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class IO_VisionReal extends SubsystemBase implements IO_VisionBase {

	private final PhotonCamera leftCamera;
	private final PhotonCamera backLeftCamera;

	private final PhotonPoseEstimator leftEstimator;
	private final PhotonPoseEstimator backLeftEstimator;

	public Matrix<N3, N1> leftMatrix;
	public Matrix<N3, N1> backLeftMatrix;

	private final AprilTagFieldLayout tagLayout;

	private Pose2d lastRobotPose = new Pose2d();

	public IO_VisionReal() {

		leftCamera = new PhotonCamera("OV2311_4");
		backLeftCamera = new PhotonCamera("OV9281_03");

		tagLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

		Transform3d leftRobotToCam =
				new Transform3d(
						CameraConstants.Camera.LEFT_CAM.translation, CameraConstants.Camera.LEFT_CAM.rotation);

		Transform3d backLeftRobotToCam =
				new Transform3d(
						CameraConstants.Camera.BACK_LEFT_CAM.translation,
						CameraConstants.Camera.BACK_LEFT_CAM.rotation);

		leftEstimator =
				new PhotonPoseEstimator(
						tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftRobotToCam);

		backLeftEstimator =
				new PhotonPoseEstimator(
						tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backLeftRobotToCam);

		leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		backLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		leftMatrix = Camera.LEFT_CAM.singleTagStdDevs;
		backLeftMatrix = Camera.BACK_LEFT_CAM.singleTagStdDevs;
	}

	@Override
	public void updateInputs(VisionInputs inputs) {

		// Get latest results
		PhotonPipelineResult leftResult = leftCamera.getLatestResult();
		PhotonPipelineResult backLeftResult = backLeftCamera.getLatestResult();

		// Check if it has targets
		inputs.hasLeftTarget = leftResult.hasTargets();
		inputs.hasBackLeftTarget = backLeftResult.hasTargets();

		// Update left camera info
		if (inputs.hasLeftTarget) {
			inputs.leftBestTargetID = leftResult.getBestTarget().getFiducialId();
			inputs.leftVisibleTagPoses =
					leftResult.getTargets().stream()
							.map(target -> tagLayout.getTagPose(target.getFiducialId()).get())
							.toArray(Pose3d[]::new);
		}

		// Update back left camera info
		if (inputs.hasBackLeftTarget) {
			inputs.backLeftBestTargetID = backLeftResult.getBestTarget().getFiducialId();
			inputs.backLeftVisibleTagPoses =
					backLeftResult.getTargets().stream()
							.map(target -> tagLayout.getTagPose(target.getFiducialId()).get())
							.toArray(Pose3d[]::new);
		}

		inputs.timestamp = leftResult.getTimestampSeconds();

		// Update pose estimation last pose
		leftEstimator.setLastPose(lastRobotPose);
		backLeftEstimator.setLastPose(lastRobotPose);

		// get current estimated poses
		Optional<EstimatedRobotPose> leftPose = getEstimatedGlobalPose(CameraConstants.Camera.LEFT_CAM);
		Optional<EstimatedRobotPose> backLeftPose =
				getEstimatedGlobalPose(CameraConstants.Camera.BACK_LEFT_CAM);

		// Check if present
		if (leftPose.isPresent()) {
			inputs.leftEstimatedPose = leftPose.get().estimatedPose.toPose2d();
			updateEstimationStdDevs(
					CameraConstants.Camera.LEFT_CAM, // Pass the camera enum instead of PhotonCamera instance
					leftPose, // Pass the Optional<EstimatedRobotPose> instead of Pose2d
					leftResult.getTargets() // Pass the targets from leftResult, not backLeftResult
					);
		} else {
			inputs.leftEstimatedPose = null;
		}

		if (backLeftPose.isPresent()) {
			inputs.backLeftEstimatedPose = backLeftPose.get().estimatedPose.toPose2d();
			updateEstimationStdDevs(
					CameraConstants.Camera.LEFT_CAM, // Pass the camera enum instead of PhotonCamera instance
					backLeftPose, // Pass the Optional<EstimatedRobotPose> instead of Pose2d
					backLeftResult.getTargets() // Pass the targets from leftResult, not backLeftResult
					);
		} else {
			inputs.backLeftEstimatedPose = null;
		}
	}

	@Override
	public void updateLastRobotPose(Pose2d currentPose) {
		lastRobotPose = currentPose;
	}

	@Override
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(CameraConstants.Camera camera) {
		PhotonPipelineResult result;
		PhotonPoseEstimator estimator;

		switch (camera) {
			case LEFT_CAM:
				result = leftCamera.getLatestResult();
				estimator = leftEstimator;
				break;
			case BACK_LEFT_CAM:
				result = backLeftCamera.getLatestResult();
				estimator = backLeftEstimator;
				break;
			default:
				return Optional.empty();
		}

		if (!result.hasTargets()) {
			return Optional.empty();
		}

		return estimator.update(result);
	}

	private void updateEstimationStdDevs(
			CameraConstants.Camera camera,
			Optional<EstimatedRobotPose> poseResult,
			List<PhotonTrackedTarget> targets) {

		// Prepare variables for the update.
		Matrix<N3, N1> defaultStdDevs;
		Matrix<N3, N1> updatedStdDevs;

		// Choose the proper default based on the camera.
		if (camera == CameraConstants.Camera.LEFT_CAM) {
			defaultStdDevs = CameraConstants.Camera.LEFT_CAM.singleTagStdDevs;
		} else if (camera == CameraConstants.Camera.BACK_LEFT_CAM) {
			defaultStdDevs = CameraConstants.Camera.BACK_LEFT_CAM.singleTagStdDevs;
		} else {
			return; // Unsupported camera type.
		}

		// If there's no pose estimate, revert to the default.
		if (poseResult.isEmpty()) {
			if (camera == CameraConstants.Camera.LEFT_CAM) {
				leftMatrix = defaultStdDevs;
			} else { // BACK_LEFT_CAM
				backLeftMatrix = defaultStdDevs;
			}
			return;
		}

		// Start with the default std devs.
		updatedStdDevs = defaultStdDevs;
		int numTags = 0;
		double totalDistance = 0.0;

		// For each detected target, accumulate distance-related data.
		for (PhotonTrackedTarget target : targets) {
			Optional<Pose3d> tagPoseOpt = tagLayout.getTagPose(target.getFiducialId());
			if (tagPoseOpt.isEmpty()) {
				continue;
			}
			numTags++;
			double distance =
					tagPoseOpt
							.get()
							.toPose2d()
							.getTranslation()
							.getDistance(poseResult.get().estimatedPose.toPose2d().getTranslation());
			totalDistance += distance;
		}

		// No valid tags detected? Revert to the default.
		if (numTags == 0) {
			if (camera == CameraConstants.Camera.LEFT_CAM) {
				leftMatrix = defaultStdDevs;
			} else {
				backLeftMatrix = defaultStdDevs;
			}
			return;
		}

		double avgDist = totalDistance / numTags;

		// When more than one tag is seen, use the multi–tag standard deviations.
		if (numTags > 1) {
			updatedStdDevs =
					(camera == CameraConstants.Camera.LEFT_CAM)
							? CameraConstants.Camera.LEFT_CAM.multiTagStdDevs
							: CameraConstants.Camera.BACK_LEFT_CAM.multiTagStdDevs;
		}

		// If only one tag is seen and it is far away, assign very high uncertainty.
		if (numTags == 1 && avgDist > 4.0) {
			updatedStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
		} else {
			updatedStdDevs = updatedStdDevs.times(1 + (avgDist * avgDist / 30));
		}

		// Finally, update the corresponding member variable.
		if (camera == CameraConstants.Camera.LEFT_CAM) {
			leftMatrix = updatedStdDevs;
		} else {
			backLeftMatrix = updatedStdDevs;
		}
	}

	public Matrix<N3, N1> getStdDev(Camera camera) {
		if (camera == Camera.LEFT_CAM) {
			return leftMatrix;
		} else if (camera == Camera.BACK_LEFT_CAM) {
			return backLeftMatrix;
		} else {
			return null;
		}
	}
}
