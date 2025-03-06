// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionIOInputsAutoLogged extends IO_VisionBase.VisionIOInputs
		implements LoggableInputs, Cloneable {
	@Override
	public void toLog(LogTable table) {
		table.put("Connected", connected);
		table.put("LatestTargetObservation", latestTargetObservation);
		table.put("PoseObservations", poseObservations);
		table.put("TagIds", tagIds);
	}

	@Override
	public void fromLog(LogTable table) {
		connected = table.get("Connected", connected);
		latestTargetObservation = table.get("LatestTargetObservation", latestTargetObservation);
		poseObservations = table.get("PoseObservations", poseObservations);
		tagIds = table.get("TagIds", tagIds);
	}

	public VisionIOInputsAutoLogged clone() {
		VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
		copy.connected = this.connected;
		copy.latestTargetObservation = this.latestTargetObservation;
		copy.poseObservations = this.poseObservations.clone();
		copy.tagIds = this.tagIds.clone();
		return copy;
	}
}
