// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IO_GyroBase {
	@AutoLog
	public static class GyroInputs {
		public boolean connected = false;
		public Rotation2d yawPosition = new Rotation2d();
		public double yawVelocityRadPerSec = 0.0;
		public double[] odometryYawTimestamps = new double[] {};
		public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
	}

	public default void updateInputs(GyroInputs inputs) {}
}
