// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.CameraConstants.Camera;
import frc.robot.vision.VisionShared.CameraData;
import java.util.*;
import org.littletonrobotics.junction.Logger;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.*;

/**
 * Real implementation of the Vision I/O interface for processing AprilTag data from multiple
 * cameras. This class handles the integration with PhotonVision and provides pose estimation for
 * robot localization.
 */
public class IO_VisionReal implements IO_VisionBase {

	// Core component storage
	private final Map<Camera, CameraData> cameraData;
	private final AprilTagFieldLayout tagLayout;
	private Pose2d lastRobotPose = new Pose2d();

	/**
	 * Initializes the vision system with all required cameras and AprilTag layout. Sets up each
	 * camera with appropriate pose estimators and calibration data.
	 */
	public IO_VisionReal() {
		// Initialize core components
		cameraData = new EnumMap<>(Camera.class);
		tagLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

		// Initialize all cameras with their specific configurations
		initializeCamera(Camera.FRONT_LEFT, "OV2311_4");
		initializeCamera(Camera.FRONT_RIGHT, "OV9281_02");
		initializeCamera(Camera.BACK_LEFT, "OV9281_03");

		initializeCamera(Camera.ELEVATED, "OV9281_01");
	}

	/**
	 * Sets up a single camera with its pose estimator and initial calibration.
	 *
	 * @param cameraType The camera position/type enum
	 * @param cameraName The device name of the camera
	 */
	private void initializeCamera(Camera cameraType, String cameraName) {
		PhotonCamera camera = new PhotonCamera(cameraName);
		Transform3d robotToCamera = new Transform3d(cameraType.translation, cameraType.rotation);

		// Configure pose estimator with multi-tag optimization
		PhotonPoseEstimator estimator =
				new PhotonPoseEstimator(
						tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
		estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		// Store all camera-related data in a single bundle
		cameraData.put(cameraType, new CameraData(camera, estimator, cameraType.singleTagStdDevs));
	}

	/**
	 * Updates vision inputs with the latest data from all cameras. This method is called in the 60Hz
	 * periodic loop.
	 *
	 * @param inputs The vision inputs structure to update
	 */
	@Override
	public void updateInputs(VisionInputs inputs) {
		// Process each camera sequentially
		for (Camera camera : Camera.values()) {
			processCamera(camera, inputs);
		}
	}

	/**
	 * Processes data from a single camera, updating pose estimates and target information.
	 *
	 * @param cameraType The camera to process
	 * @param inputs The vision inputs to update
	 */
	private void processCamera(Camera cameraType, VisionInputs inputs) {

		long startTime = System.nanoTime();

		// Get the camera data object
		CameraData data = cameraData.get(cameraType);

		// Get the latest result from the camera
		PhotonPipelineResult result = data.camera.getLatestResult();

		// Update estimator with latest robot pose for better accuracy
		data.estimator.setLastPose(lastRobotPose);

		// Check if camera result has a target
		boolean hasTarget = result.hasTargets();

		// Update hasTarget input data
		VisionShared.setHasTarget(inputs, cameraType, hasTarget);

		// Update more target input data
		VisionShared.updateTargetInfo(inputs, cameraType, result, data, tagLayout, lastRobotPose);

		EstimatedRobotPose estimatedPose = data.estimator.update(result).orElse(null);
		if (estimatedPose != null) {
			VisionShared.setPoseEstimate(inputs, cameraType, estimatedPose.estimatedPose.toPose2d());
			VisionShared.updateEstimationStdDevs(
					cameraType, estimatedPose, result.getTargets(), data, tagLayout);
		}

		long getResultTime = System.nanoTime();

		// Log specific operation timings
		Logger.recordOutput("Vision/" + cameraType + "/GetResultMS", (getResultTime - startTime) / 1e6);
	}

	@Override
	public void updateLastRobotPose(Pose2d currentPose) {
		lastRobotPose = currentPose;
	}

	@Override
	public Matrix<N3, N1> getStdDev(Camera camera) {
		return cameraData.get(camera).stdDevMatrix;
	}
}
