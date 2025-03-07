// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
	// AprilTag layout
	public static AprilTagFieldLayout aprilTagLayout =
			AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

	// Front Camera
	public static String camera0Name = "OV9281_02"; // Front Camera
	public static Transform3d robotToCamera0 =
			new Transform3d(
					new Translation3d( // X (red), Y (green), Z (height)
							Units.inchesToMeters(12.5), Units.inchesToMeters(0), Units.inchesToMeters(7.5)),
					new Rotation3d(
							Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(0)));

	public static String camera1Name = "OV9281_03"; // Back Camera
	public static Transform3d robotToCamera1 =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(-11.25 - 2.0),
							Units.inchesToMeters(9),
							Units.inchesToMeters(19.5)),
					new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians((-155 - 15)), 0));

	// Front Left Camera
	public static String camera2Name = "OV2311_4"; // Front Left Camera
	public static Transform3d robotToCamera2 =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(2.0), Units.inchesToMeters(9.5), Units.inchesToMeters(19)),
					new Rotation3d(0, Units.degreesToRadians(-15.0), Units.degreesToRadians(45)));

	// Front Right Camera
	public static String camera3Name = "OV2311_5"; // Front Right Camera
	public static Transform3d robotToCamera3 =
			new Transform3d(
					new Translation3d(
							Units.inchesToMeters(2.0), Units.inchesToMeters(-9.5), Units.inchesToMeters(19)),
					new Rotation3d(
							Units.degreesToRadians(270 + 45),
							Units.degreesToRadians(-15.0),
							Units.degreesToRadians(-45)));

	// Basic filtering thresholds
	public static double maxAmbiguity = 0.3;
	public static double maxZError = 0.75;

	// Standard deviation baselines, for 1 meter distance and 1 tag
	// (Adjusted automatically based on distance and # of tags)
	public static double linearStdDevBaseline = 0.02; // Meters
	public static double angularStdDevBaseline = 0.06; // Radians

	// Standard deviation multipliers for each camera
	// (Adjust to trust some cameras more than others)
	public static double[] cameraStdDevFactors =
			new double[] {
				1.0, // Camera 0 (Front Camera)
				1.0, // Camera 1 (Back Camera)
				1.0, // Camera 2 (Front Left Camera)
				1.0 // Camera 3 (Front Right Camera)
			};

	// Multipliers to apply for MegaTag 2 observations
	public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
	public static double angularStdDevMegatag2Factor =
			Double.POSITIVE_INFINITY; // No rotation data available
}
