// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.constants.RobotConstants;

public class IO_ClimbReal implements IO_ClimbBase {

	private SparkFlex climbMotor;

	public IO_ClimbReal() {
		climbMotor = new SparkFlex(RobotConstants.Climb.CLIMB_MOTOR_ID, MotorType.kBrushless);

		// Configure the climb motor
		SparkFlexConfig climbSparkFlexConfig = new SparkFlexConfig();
		climbSparkFlexConfig.smartCurrentLimit(RobotConstants.Climb.CLIMB_MOTOR_CURRENT_LIMIT);
		climbSparkFlexConfig.idleMode(IdleMode.kBrake);
		climbMotor.configure(
				climbSparkFlexConfig,
				SparkBase.ResetMode.kNoResetSafeParameters,
				SparkBase.PersistMode.kPersistParameters);

		// Reset the encoder position
		climbMotor.getEncoder().setPosition(0);
	}

	@Override
	public void updateInputs(ClimbInputs inputs) {
		inputs.motorCurrent = climbMotor.getOutputCurrent();
		inputs.motorPosition = climbMotor.getEncoder().getPosition();
		inputs.motorVelocity = climbMotor.getEncoder().getVelocity();
	}

	@Override
	public void setMotorSpeed(double speed) {
		climbMotor.set(speed);
	}
}
