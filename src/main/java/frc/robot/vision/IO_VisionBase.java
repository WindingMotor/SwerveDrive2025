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
import frc.robot.constants.CameraConstants.Camera;
import org.littletonrobotics.junction.AutoLog;

public interface IO_VisionBase {
	@AutoLog
	public static class VisionInputs {

		public boolean flHasTarget = false;
		public boolean frHasTarget = false;

		public boolean blHasTarget = false;
		public boolean elHasTarget = false;

		// public double flLatencyMS = 0.0;
		// public double frLatencyMS = 0.0;

		//	public double blLatencyMS = 0.0;
		//	public double elLatencyMS = 0.0;

		public double flBestTargetID = -1.0;
		public double frBestTargetID = -1.0;

		public double blBestTargetID = -1.0;
		public double elBestTargetID = -1.0;

		public Pose3d[] flVisibleTagPoses = new Pose3d[0];
		public Pose3d[] frVisibleTagPoses = new Pose3d[0];

		public Pose3d[] blVisibleTagPoses = new Pose3d[0];
		public Pose3d[] elVisibleTagPoses = new Pose3d[0];

		public Pose2d flEstimatedPose = new Pose2d();
		public Pose2d frEstimatedPose = new Pose2d();

		public Pose2d blEstimatedPose = new Pose2d();
		public Pose2d elEstimatedPose = new Pose2d();

		public double flTimestamp = 0.0;
		public double frTimestamp = 0.0;

		public double blTimestamp = 0.0;
		public double elTimestamp = 0.0;
	}

	public void updateInputs(VisionInputs inputs);

	public void updateLastRobotPose(Pose2d currentPose);

	public Matrix<N3, N1> getStdDev(Camera camera);
}
