// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface IO_VisionBase {
	@AutoLog
	public static class VisionInputs {
		public double timestamp;

		public boolean hasLeftTarget = false;
		public boolean hasFrontLeftTopTarget = false;
		public boolean hasBackLeftTarget = false;

		public double leftLatencyMS = 0.0;
		public double frontLeftTopMS = 0.0;
		public double backLeftLatencyMS = 0.0;

		public double leftBestTargetID = -1.0;
		public double frontLeftTopBestTargetID = -1.0;
		public double backLeftBestTargetID = -1.0;

		public Pose3d[] leftVisibleTagPoses = new Pose3d[0];
		public Pose3d[] rightVisibleTagPoses = new Pose3d[0];
		public Pose3d[] backLeftVisibleTagPoses = new Pose3d[0];

		public Pose3d lastEstimatedPose = new Pose3d();
	}

	public void updateInputs(VisionInputs inputs);

	public void updatePoseEstimation(Pose2d currentPose);

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose();
}
