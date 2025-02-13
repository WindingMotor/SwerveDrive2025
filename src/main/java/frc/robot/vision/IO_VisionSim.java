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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.CameraConstants.Camera;
import frc.robot.vision.VisionShared.CameraData;
import java.util.*;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Simulated implementation of the Vision I/O interface for processing AprilTag data from multiple
 * cameras. This class simulates camera behavior and pose estimation for robot localization in a
 * virtual environment.
 */
public class IO_VisionSim implements IO_VisionBase {

	// Core component storage
	private final Map<Camera, CameraData> cameraData;
	private final VisionSystemSim visionSim;
	private final AprilTagFieldLayout tagLayout;
	private Pose2d lastRobotPose = new Pose2d();

	/**
	 * Initializes the simulated vision system with all required cameras and AprilTag layout. Sets up
	 * each camera with appropriate pose estimators and simulation properties.
	 */
	public IO_VisionSim() {
		// Initialize core components
		cameraData = new EnumMap<>(Camera.class);
		tagLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
		visionSim = new VisionSystemSim("Vision");

		// Add all AprilTags from the layout to the vision simulation
		visionSim.addAprilTags(tagLayout);

		// Set camera properties with more realistic simulation settings
		SimCameraProperties properties = new SimCameraProperties();
		properties.setCalibration(1280, 900, new Rotation2d(Math.toRadians(70)));
		properties.setCalibError(0.35, 0.08); // Pixel detection error
		properties.setFPS(25);
		properties.setAvgLatencyMs(35);
		properties.setLatencyStdDevMs(5);

		// Initialize cameras with proper NetworkTables entries and simulation setup
		for (Camera cameraType : Camera.values()) {
			// Create PhotonCamera with unique NetworkTables path
			PhotonCamera camera = new PhotonCamera(NetworkTableInstance.getDefault(), cameraType.name);

			// Create camera simulator with properties
			PhotonCameraSim cameraSim = new PhotonCameraSim(camera, properties);

			// Camera streams and wireframe for debugging
			cameraSim.enableRawStream(true);
			cameraSim.enableProcessedStream(true);
			cameraSim.enableDrawWireframe(true);

			// Define camera's transform relative to robot
			Transform3d robotToCamera = new Transform3d(cameraType.translation, cameraType.rotation);

			// Add camera to vision simulation
			visionSim.addCamera(cameraSim, robotToCamera);

			// Configure pose estimator with multi-tag optimization
			PhotonPoseEstimator estimator =
					new PhotonPoseEstimator(
							tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
			estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

			// Store all camera-related data in a single bundle
			cameraData.put(
					cameraType, new CameraData(camera, cameraSim, estimator, cameraType.singleTagStdDevs));
		}
	}

	/**
	 * Updates the last known robot pose for simulation purposes. This is crucial for generating
	 * accurate simulated vision data.
	 */
	@Override
	public void updateLastRobotPose(Pose2d currentPose) {
		lastRobotPose = currentPose;
	}

	/**
	 * Updates vision inputs with the latest data from all cameras. This method is called in the 60Hz
	 * periodic loop.
	 *
	 * @param inputs The vision inputs structure to update
	 */
	@Override
	public void updateInputs(VisionInputs inputs) {
		// Ensure vision simulation is updated with the latest robot pose.
		// This is crucial for generating accurate simulated vision data.
		visionSim.update(new Pose3d(lastRobotPose));

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
		// Get the camera data object
		CameraData data = cameraData.get(cameraType);

		// Update camera results
		List<PhotonPipelineResult> results = data.camera.getAllUnreadResults();
		if (!results.isEmpty()) {
			// Sort and get the most recent result
			results.sort((a, b) -> Double.compare(b.getTimestampSeconds(), a.getTimestampSeconds()));
			data.latestResult = results.get(0);
		}

		// Check if camera result has a target
		boolean hasTarget = data.latestResult.hasTargets();

		// Update hasTarget input data
		VisionShared.setHasTarget(inputs, cameraType, hasTarget);

		// Update more target input data
		VisionShared.updateTargetInfo(
				inputs, cameraType, data.latestResult, data, tagLayout, lastRobotPose);

		// Update closest target info
		VisionShared.updateClosestTargetInfo(
				inputs, cameraType, data.latestResult, lastRobotPose, tagLayout);

		// Attempt to get estimated pose
		EstimatedRobotPose estimatedPose = getEstimatedGlobalPose(cameraType).orElse(null);
		if (estimatedPose != null) {
			VisionShared.setPoseEstimate(inputs, cameraType, estimatedPose.estimatedPose.toPose2d());
			VisionShared.updateEstimationStdDevs(
					cameraType, estimatedPose, data.latestResult.getTargets(), data, tagLayout);
		}
	}

	/** Retrieves the estimated global pose for a specific camera. */
	private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Camera camera) {
		CameraData data = cameraData.get(camera);
		PhotonPipelineResult result = data.latestResult;

		if (!result.hasTargets()) {
			return Optional.empty();
		}

		// Use the pose estimator to get the estimated pose
		return data.estimator.update(result);
	}

	/** Retrieves the standard deviation matrix for a specific camera. */
	@Override
	public Matrix<N3, N1> getStdDev(Camera camera) {
		return cameraData.get(camera).stdDevMatrix;
	}
}
