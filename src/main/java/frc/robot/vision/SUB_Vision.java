// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.CameraConstants.Camera;
import org.littletonrobotics.junction.Logger;

public class SUB_Vision extends SubsystemBase {
	private final IO_VisionBase io;
	public final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

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
	public void updateLastRobotPose(Pose2d currentPose) {
		io.updateLastRobotPose(currentPose);
	}

	public Pose2d getCameraPose(CameraConstants.Camera camera) {
		if (camera == Camera.LEFT_CAM && inputs.leftEstimatedPose != null) {
			return inputs.leftEstimatedPose;
		} else if (camera == Camera.BACK_LEFT_CAM && inputs.backLeftEstimatedPose != null) {
			return inputs.backLeftEstimatedPose;
		} else {
			return null;
		}
	}

	public Matrix<N3, N1> getStdDev(Camera camera) {

		return io.getStdDev(camera);
	}

	public boolean hasTargets() {
		return inputs.hasLeftTarget || inputs.hasBackLeftTarget;
	}
}
