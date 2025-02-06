// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.CameraConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class IO_VisionSim implements IO_VisionBase {
	private final VisionSystemSim visionSim;
	private final Map<CameraConstants.Camera, PhotonCameraSim> cameraSims = new HashMap<>();
	private final Map<CameraConstants.Camera, PhotonPipelineResult> currentResults = new HashMap<>();
	private Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();
	private final AprilTagFieldLayout fieldLayout;

	private Pose3d lastCurrentPose = new Pose3d();

	public IO_VisionSim() {
		fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
		visionSim = new VisionSystemSim("Vision");
		visionSim.addAprilTags(fieldLayout);

		// Set camera properties
		SimCameraProperties properties = new SimCameraProperties();
		properties.setCalibration(960, 720, new Rotation2d(Math.toRadians(100)));
		properties.setCalibError(0.25, 0.08);
		properties.setFPS(30);
		properties.setAvgLatencyMs(35);
		properties.setLatencyStdDevMs(5);

		// Initialize cameras with proper NetworkTables entries
		for (CameraConstants.Camera cam : CameraConstants.Camera.values()) {
			PhotonCamera camera = new PhotonCamera(NetworkTableInstance.getDefault(), cam.name);
			PhotonCameraSim cameraSim = new PhotonCameraSim(camera, properties);
			cameraSim.enableDrawWireframe(true);
			cameraSims.put(cam, cameraSim);
			currentResults.put(cam, new PhotonPipelineResult());

			Transform3d robotToCam = new Transform3d(cam.translation, cam.rotation);
			visionSim.addCamera(cameraSim, robotToCam);
		}
	}

	@Override
	public void updateInputs(VisionInputs inputs) {
		List<Pose3d> leftTagPoses = new ArrayList<>();
		List<Pose3d> frontLeftTopPoses = new ArrayList<>();
		List<Pose3d> backLeftTagPoses = new ArrayList<>();

		// Update all camera results first
		for (Map.Entry<CameraConstants.Camera, PhotonCameraSim> entry : cameraSims.entrySet()) {
			CameraConstants.Camera cam = entry.getKey();
			PhotonCamera camera = entry.getValue().getCamera();

			List<PhotonPipelineResult> results = camera.getAllUnreadResults();
			if (!results.isEmpty()) {
				// Get the most recent result
				results.sort((a, b) -> Double.compare(b.getTimestampSeconds(), a.getTimestampSeconds()));
				currentResults.put(cam, results.get(0));
			}
		}

		// Process the results for each camera
		for (Map.Entry<CameraConstants.Camera, PhotonCameraSim> entry : cameraSims.entrySet()) {
			CameraConstants.Camera cam = entry.getKey();
			PhotonPipelineResult result = currentResults.get(cam);

			if (result.hasTargets()) {
				PhotonTrackedTarget bestTarget = result.getBestTarget();

				// Calculate camera position based on robot pose
				Pose3d cameraPose =
						lastCurrentPose.transformBy(new Transform3d(cam.translation, cam.rotation));

				// Get poses for the current camera
				for (PhotonTrackedTarget target : result.getTargets()) {
					Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
					if (tagPose.isPresent()) {
						switch (cam) {
							case LEFT_CAM:
								leftTagPoses.add(tagPose.get());
								leftTagPoses.add(cameraPose);
								inputs.hasLeftTarget = true;
								inputs.leftBestTargetID = bestTarget.getFiducialId();
								break;
							case FRONT_LEFT_TOP_CAM:
								frontLeftTopPoses.add(tagPose.get());
								frontLeftTopPoses.add(cameraPose);
								inputs.hasFrontLeftTopTarget = true;
								inputs.frontLeftTopBestTargetID = bestTarget.getFiducialId();
								break;
							case BACK_LEFT_CAM:
								backLeftTagPoses.add(tagPose.get());
								backLeftTagPoses.add(cameraPose);
								inputs.hasBackLeftTarget = true;
								inputs.backLeftBestTargetID = bestTarget.getFiducialId();
								break;
						}
					}
				}
			}
		}

		inputs.leftVisibleTagPoses = leftTagPoses.toArray(new Pose3d[0]);
		inputs.rightVisibleTagPoses = frontLeftTopPoses.toArray(new Pose3d[0]);
		inputs.backLeftVisibleTagPoses = backLeftTagPoses.toArray(new Pose3d[0]);
		inputs.lastEstimatedPose =
				lastEstimatedPose.isPresent() ? lastEstimatedPose.get().estimatedPose : null;
	}

	@Override
	public void updatePoseEstimation(Pose2d currentPose) {
		lastCurrentPose = new Pose3d(currentPose);
		visionSim.update(currentPose);
	}

	@Override
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
		return lastEstimatedPose;
	}
}
