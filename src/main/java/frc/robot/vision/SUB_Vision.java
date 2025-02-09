// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CameraConstants.Camera;
import frc.robot.vision.VisionShared.CameraEstimationData;
import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class SUB_Vision extends SubsystemBase {
	private final IO_VisionBase io;
	public final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

	// Cache for camera estimation data
	private final Map<Camera, CameraEstimationData> estimationCache;
	private final Map<Camera, Boolean> cacheValid;

	public SUB_Vision(IO_VisionBase io) {
		this.io = io;
		this.estimationCache = new EnumMap<>(Camera.class);
		this.cacheValid = new EnumMap<>(Camera.class);

		// Initialize cache validity flags
		for (Camera camera : Camera.values()) {
			cacheValid.put(camera, false);
		}
	}

	@Override
	public void periodic() {
		// Update inputs
		io.updateInputs(inputs);

		// Invalidate all caches since we have new input data
		for (Camera camera : Camera.values()) {
			cacheValid.put(camera, false);
		}

		// Process inputs
		Logger.processInputs("Vision", inputs);
	}

	public void updateLastRobotPose(Pose2d currentPose) {
		io.updateLastRobotPose(currentPose);
	}

	public Optional<CameraEstimationData> getCameraEstimationData(Camera camera) {
		// If the cache is valid, (aka no new input data) return the last cached data
		if (cacheValid.get(camera) && estimationCache.containsKey(camera)) {
			return Optional.of(estimationCache.get(camera));
		}

		// Otherwise, compute new estimation data
		Pose2d pose;
		double timestamp;

		switch (camera) {
			case FRONT_LEFT:
				if (inputs.flEstimatedPose == null) return Optional.empty();
				pose = inputs.flEstimatedPose;
				timestamp = inputs.flTimestamp;
				break;
			case BACK_LEFT:
				if (inputs.blEstimatedPose == null) return Optional.empty();
				pose = inputs.blEstimatedPose;
				timestamp = inputs.blTimestamp;
				break;
			case ELEVATED:
				if (inputs.elEstimatedPose == null) return Optional.empty();
				pose = inputs.elEstimatedPose;
				timestamp = inputs.elTimestamp;
				break;
			case FRONT_RIGHT:
				if (inputs.frEstimatedPose == null) return Optional.empty();
				pose = inputs.frEstimatedPose;
				timestamp = inputs.frTimestamp;
				break;
			default:
				return Optional.empty();
		}

		// Create new estimation data and cache it
		CameraEstimationData newData = new CameraEstimationData(pose, timestamp, io.getStdDev(camera));
		estimationCache.put(camera, newData);
		cacheValid.put(camera, true);

		return Optional.of(newData);
	}
}
