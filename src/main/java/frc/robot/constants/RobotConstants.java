// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class RobotConstants {

	public enum RobotMode {
		REAL, // Physical robot hardware
		SIM, // Simulation mode
		REPLAY // Replay mode for log analysis
	}

	public static final RobotMode ROBOT_MODE = RobotMode.REAL;

	public static final double BATTERY_VOLTAGE_CRITICAL = 10.5; // Volts
	public static final double BATTERY_VOLTAGE_WARNING = 11.5; // Volts

	public static final boolean FORCE_REDUX_SERVER_ON = false;

	public static final double ROBOT_MASS = 65.000; // kg
	public static final Matter CHASSIS =
			new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
	public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
	public static final double MAX_SPEED = 5.450; // m/s

	public final class Intake {

		public static final int ARM_MOTOR_ID = 21;
		public static final int ARM_MOTOR_CURRENT_LIMIT = 25;

		public static final int ARM_ENCODER_LOOP_OFFSET = 12;
		public static final int ARM_ENCODER_PID_OFFSET = -14;
		public static final int ARM_ENCODER_FACTOR = 165;

		public static final double ARM_P = 0.013;
		public static final double ARM_I = 0.0;
		public static final double ARM_D = 0.0;

		public static final int WHEEL_MOTOR_ID = 22;
		public static final int WHEEL_MOTOR_CURRENT_LIMIT = 55;

		public static final int SENSOR_RIO_ID = 9;
	}

	public final class Elevator {

		public static final int LEFT_MOTOR_ID = 10;
		public static final int RIGHT_MOTOR_ID = 9;
		public static final String CANIVORE_NAME = "canivore";

		public static final double METERS_PER_MOTOR_ROTATION = 0.0257951242902;

		// Motion Magic Configuration
		public static final double CRUISE_VELOCITY = 2500; // mm/s
		public static final double ACCELERATION = 4000; // mm/s²
		public static final double JERK = 6000; // mm/s³

		// Feed Forward and PID Constants
		public static final double KS = 0.35; // Static Friction Voltage
		public static final double KV = 0.01; // Velocity Feed Forward
		public static final double KA = 0.00; // Acceleration Feed Forward
		public static final double KG = 0.22; // Gravity Compensation

		public static final double KP = 0.52; // Position error gain (V per meter)
		public static final double KI = 0.0; // Integral gain for steady-state error
		public static final double KD = 0.0; // Derivative gain for damping

		// Position Limits
		public static final double MIN_HEIGHT = 0.0; // mm
		public static final double MAX_HEIGHT = 2420.0; // mm
	}
}
