// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FieldConstants {

	/* OLD Non-Dyanmic Scoring positions
	public static final Pose2d BLUE_TOP_TOP_LEFT = new Pose2d(6, 3.940, Rotation2d.fromDegrees(-90));
	public static final Pose2d BLUE_TOP_TOP_RIGHT = new Pose2d(6, 3.620, Rotation2d.fromDegrees(-90));

	public static final Pose2d BLUE_TOP_LEFT_TOP =
			new Pose2d(5.586, 5.1, Rotation2d.fromDegrees(-30));
	public static final Pose2d BLUE_TOP_LEFT_BOTTOM =
			new Pose2d(5.308, 5.268, Rotation2d.fromDegrees(-30));
	*/

	public static final double fieldLength = Units.inchesToMeters(690.876);
	public static final double fieldWidth = Units.inchesToMeters(317);
	public static final double startingLineX =
			Units.inchesToMeters(299.438); // Measured from the inside of starting line
	public static final double algaeDiameter = Units.inchesToMeters(16);

	public static class Processor {
		public static final Pose2d centerFace =
				new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
	}

	public static class Barge {
		public static final Translation2d farCage =
				new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
		public static final Translation2d middleCage =
				new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
		public static final Translation2d closeCage =
				new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

		// Measured from floor to bottom of cage
		public static final double deepHeight = Units.inchesToMeters(3.125);
		public static final double shallowHeight = Units.inchesToMeters(30.125);
	}

	public static class CoralStation {
		public static final Pose2d leftCenterFace =
				new Pose2d(
						Units.inchesToMeters(33.526),
						Units.inchesToMeters(291.176),
						Rotation2d.fromDegrees(90 - 144.011));
		public static final Pose2d rightCenterFace =
				new Pose2d(
						Units.inchesToMeters(33.526),
						Units.inchesToMeters(25.824),
						Rotation2d.fromDegrees(144.011 - 90));
	}

	public static class Reef {
		public static final Translation2d center =
				new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));

		public static final double faceToZoneLine =
				Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

		public static final Pose2d[] centerFaces =
				new Pose2d[6]; // Starting facing the driver station in clockwise order
		public static final List<Map<ReefHeight, Pose3d>> branchPositions =
				new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

		static {
			// Initialize faces
			centerFaces[0] =
					new Pose2d(
							Units.inchesToMeters(144.003),
							Units.inchesToMeters(158.500),
							Rotation2d.fromDegrees(180));
			centerFaces[1] =
					new Pose2d(
							Units.inchesToMeters(160.373),
							Units.inchesToMeters(186.857),
							Rotation2d.fromDegrees(120));
			centerFaces[2] =
					new Pose2d(
							Units.inchesToMeters(193.116),
							Units.inchesToMeters(186.858),
							Rotation2d.fromDegrees(60));
			centerFaces[3] =
					new Pose2d(
							Units.inchesToMeters(209.489),
							Units.inchesToMeters(158.502),
							Rotation2d.fromDegrees(0));
			centerFaces[4] =
					new Pose2d(
							Units.inchesToMeters(193.118),
							Units.inchesToMeters(130.145),
							Rotation2d.fromDegrees(-60));
			centerFaces[5] =
					new Pose2d(
							Units.inchesToMeters(160.375),
							Units.inchesToMeters(130.144),
							Rotation2d.fromDegrees(-120));

			// Initialize branch positions
			for (int face = 0; face < 6; face++) {
				Map<ReefHeight, Pose3d> fillRight = new HashMap<ReefHeight, Pose3d>();
				Map<ReefHeight, Pose3d> fillLeft = new HashMap<ReefHeight, Pose3d>();
				for (var level : ReefHeight.values()) {
					Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
					double adjustX = Units.inchesToMeters(30.738);
					double adjustY = Units.inchesToMeters(6.469);

					fillRight.put(
							level,
							new Pose3d(
									new Translation3d(
											poseDirection
													.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
													.getX(),
											poseDirection
													.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
													.getY(),
											level.height),
									new Rotation3d(
											0,
											Units.degreesToRadians(level.pitch),
											poseDirection.getRotation().getRadians())));
					fillLeft.put(
							level,
							new Pose3d(
									new Translation3d(
											poseDirection
													.transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
													.getX(),
											poseDirection
													.transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
													.getY(),
											level.height),
									new Rotation3d(
											0,
											Units.degreesToRadians(level.pitch),
											poseDirection.getRotation().getRadians())));
				}
				branchPositions.add(fillRight);
				branchPositions.add(fillLeft);
			}
		}
	}

	public static class StagingPositions {
		// Measured from the center of the ice cream
		public static final Pose2d leftIceCream =
				new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
		public static final Pose2d middleIceCream =
				new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
		public static final Pose2d rightIceCream =
				new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
	}

	public enum ReefHeight {
		L4(Units.inchesToMeters(72), -90),
		L3(Units.inchesToMeters(47.625), -35),
		L2(Units.inchesToMeters(31.875), -35),
		L1(Units.inchesToMeters(18), 0);

		ReefHeight(double height, double pitch) {
			this.height = height;
			this.pitch = pitch; // in degrees
		}

		public final double height;
		public final double pitch;
	}
}
