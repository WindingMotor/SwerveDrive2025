// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

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

	public static String camera1Name = "camera_1";
	public static Transform3d robotToCamera1 =
			new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

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
				1.0, // Camera 0
				1.0 // Camera 1
			};

	// Multipliers to apply for MegaTag 2 observations
	public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
	public static double angularStdDevMegatag2Factor =
			Double.POSITIVE_INFINITY; // No rotation data available
}
