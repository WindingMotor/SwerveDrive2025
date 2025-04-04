// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

public final class RobotConstants {

	public enum RobotMode {
		REAL, // Physical robot hardware
		SIM, // Simulation mode
		REPLAY // Replay mode for log analysis
	}

	public static final RobotMode ROBOT_MODE = RobotMode.REAL;

	public static final double BATTERY_VOLTAGE_CRITICAL = 10.5; // Volts
	public static final double BATTERY_VOLTAGE_WARNING = 11.5; // Volts

	public final class Intake {

		public static final int ARM_MOTOR_ID = 21;
		public static final int ARM_MOTOR_CURRENT_LIMIT = 35;

		public static final double ARM_ENCODER_LOOP_OFFSET = -8.6; // (-35 + 15) + 11;
		public static final double ARM_ENCODER_PID_OFFSET = 7.0;

		// (((-35.0 + 15.0) + 22.51) + 15 + .8) - 2.0; // -14

		public static final int ARM_ENCODER_FACTOR = 165;

		public static final double ARM_P = 0.013;
		public static final double ARM_I = 0.0;
		public static final double ARM_D = 0.0;

		public static final int WHEEL_MOTOR_ID = 22;
		public static final int WHEEL_MOTOR_CURRENT_LIMIT_NORMAL = 55;
		public static final int WHEEL_MOTOR_CURRENT_LIMIT_LOWER = 5;

		public static final int SENSOR_RIO_ID = 9;
	}

	public final class Elevator {

		public static final int LEFT_MOTOR_ID = 10;
		public static final int RIGHT_MOTOR_ID = 9;
		public static final String CANIVORE_NAME = "canivore";

		public static final double METERS_PER_MOTOR_ROTATION = 0.025795;

		// Motion Magic Configuration
		public static final double CRUISE_VELOCITY = 2 / METERS_PER_MOTOR_ROTATION; // m/s
		public static final double ACCELERATION = 1.5 / METERS_PER_MOTOR_ROTATION; // m/s²
		public static final double JERK = .5 / METERS_PER_MOTOR_ROTATION; // m/s³

		// Feed Forward and PID Constants
		public static final double KS = 0.56134; // Add V output to overcome static friction
		// 0.56134

		public static final double KV =
				0.089361; // 0.089361 Velocity of 1 m/s results in 0.01 V output //  0.085;
		// 0.08

		public static final double KA = 0.0; // 0.042142 Accel of 1 m/s² results in 0.00 V output

		public static final double KG = 0.3; // Gravity Compensation
		// 0.54418

		public static final double KP =
				1.5; // 1.5 //   34.742 A position error of 1m results in 12 V output

		// 1.0
		public static final double KI = 0; // 1.6658 No output for integrated error
		public static final double KD = 0.0; //  A velocity error of 1 m/s results in 0.1 V output

		// Position Limits
		public static final double MIN_HEIGHT = 0.0; // m
		public static final double MAX_HEIGHT = 1.85; // m
	}

	public final class Climb {

		public static final int CLIMB_MOTOR_ID = 23;
		public static final int CLIMB_MOTOR_CURRENT_LIMIT = 45;
	}
}
