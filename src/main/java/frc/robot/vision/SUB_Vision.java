// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class SUB_Vision extends SubsystemBase {
	private final IO_VisionBase io;
	private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

	public SUB_Vision(IO_VisionBase io) {
		this.io = io;
	}

	@Override
	public void periodic() {

		// Update inputs
		io.updateInputs(inputs);

		// Process inputs
		Logger.processInputs("Vision", inputs);
	}

	// This will be called by the Swerve Drive subsystem to update the estimated pose.
	public void updatePoseEstimation(Pose2d currentPose) {
		io.updatePoseEstimation(currentPose);
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
		return io.getEstimatedGlobalPose();
	}

	public boolean hasTargets() {
		return inputs.hasLeftTarget || inputs.hasFrontLeftTopTarget || inputs.hasBackLeftTarget;
	}
}
