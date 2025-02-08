// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.CameraConstants.Camera;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface IO_VisionBase {
	@AutoLog
	public static class VisionInputs {
		public double timestamp;

		public boolean hasLeftTarget = false;
		public boolean hasBackLeftTarget = false;

		public double leftLatencyMS = 0.0;
		public double backLeftLatencyMS = 0.0;

		public double leftBestTargetID = -1.0;
		public double backLeftBestTargetID = -1.0;

		public Pose3d[] leftVisibleTagPoses = new Pose3d[0];
		public Pose3d[] backLeftVisibleTagPoses = new Pose3d[0];

		public Pose2d leftEstimatedPose = new Pose2d();
		public Pose2d backLeftEstimatedPose = new Pose2d();
	}

	public void updateInputs(VisionInputs inputs);

	public void updateLastRobotPose(Pose2d currentPose);

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(CameraConstants.Camera camera);

	public Matrix<N3, N1> getStdDev(Camera camera);
}
