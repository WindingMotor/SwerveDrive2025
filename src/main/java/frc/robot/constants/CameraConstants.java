// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class CameraConstants {
	public static final double MAXIMUM_AMBIGUITY = 0.25;

	public enum Camera {
		FRONT_LEFT(
				"OV2311_4",
				new Rotation3d(0, Math.toRadians(-24.094 + 15), Math.toRadians(75 + 45)),
				new Translation3d(
						Units.inchesToMeters(-2.0),
						Units.inchesToMeters(9.5),
						Units.inchesToMeters(19)), // X (red), Y (green), Z (height)
				VecBuilder.fill(1.0, 1.0, 4),
				VecBuilder.fill(0.5, 0.5, 1)),

		ELEVATED(
				"OV9281_0222",
				new Rotation3d(0, Units.degreesToRadians(180 - 30), Units.degreesToRadians(-35)),
				new Translation3d(
						Units.inchesToMeters(-5.5),
						Units.inchesToMeters(-11.5),
						Units.inchesToMeters(38.5)), // X (red), Y (green), Z (height)
				VecBuilder.fill(1.0, 1.0, 4),
				VecBuilder.fill(0.5, 0.5, 1)),

		BACK_LEFT(
				"OV9281_03",
				new Rotation3d(0, Units.degreesToRadians((-155 - 15) + 180 - 15), 0),
				new Translation3d(
						Units.inchesToMeters(11.25), Units.inchesToMeters(-9), Units.inchesToMeters(19.5)),
				VecBuilder.fill(1.0, 1.0, 4),
				VecBuilder.fill(0.5, 0.5, 1)),

		FRONT_RIGHT(
				"OV9281_02",
				new Rotation3d(0, Math.toRadians(-24.094 + 15), Math.toRadians(-75 - 45)),
				new Translation3d(
						Units.inchesToMeters(-2.0),
						Units.inchesToMeters(-9.5),
						Units.inchesToMeters(19)), // X (red), Y (green), Z (height)
				VecBuilder.fill(1.0, 1.0, 4),
				VecBuilder.fill(0.5, 0.5, 1));

		public final String name;
		public final Rotation3d rotation;
		public final Translation3d translation;
		public final Matrix<N3, N1> singleTagStdDevs;
		public final Matrix<N3, N1> multiTagStdDevs;

		Camera(
				String name,
				Rotation3d rotation,
				Translation3d translation,
				Matrix<N3, N1> singleTagStdDevs,
				Matrix<N3, N1> multiTagStdDevs) {
			this.name = name;
			this.rotation = rotation;
			this.translation = translation;
			this.singleTagStdDevs = singleTagStdDevs;
			this.multiTagStdDevs = multiTagStdDevs;
		}
	}

	public static final Pose3d[] CAMERA_POSITIONS = {
		new Pose3d(Camera.FRONT_LEFT.translation, Camera.FRONT_LEFT.rotation),
		new Pose3d(Camera.ELEVATED.translation, Camera.ELEVATED.rotation),
		new Pose3d(Camera.BACK_LEFT.translation, Camera.BACK_LEFT.rotation),
		new Pose3d(Camera.FRONT_RIGHT.translation, Camera.FRONT_RIGHT.rotation)
	};
}
