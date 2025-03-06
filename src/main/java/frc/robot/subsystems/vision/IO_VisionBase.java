// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IO_VisionBase {
	@AutoLog
	public static class VisionIOInputs {
		public boolean connected = false;
		public TargetObservation latestTargetObservation =
				new TargetObservation(new Rotation2d(), new Rotation2d());
		public PoseObservation[] poseObservations = new PoseObservation[0];
		public int[] tagIds = new int[0];
	}

	/** Represents the angle to a simple target, not used for pose estimation. */
	public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

	/** Represents a robot pose sample used for pose estimation. */
	public static record PoseObservation(
			double timestamp,
			Pose3d pose,
			double ambiguity,
			int tagCount,
			double averageTagDistance,
			PoseObservationType type) {}

	public static enum PoseObservationType {
		MEGATAG_1,
		MEGATAG_2,
		PHOTONVISION
	}

	public default void updateInputs(VisionIOInputs inputs) {}
}
