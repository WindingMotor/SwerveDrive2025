// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.RobotConstants;

public class IO_ElevatorReal implements IO_ElevatorBase {

	private final TalonFX leftMotor_10;
	private final TalonFX rightMotor_9;
	private final MotionMagicExpoVoltage magicMotion;

	public IO_ElevatorReal() {
		leftMotor_10 = new TalonFX(10, "canivore");
		rightMotor_9 = new TalonFX(9, "canivore");
		var motorConfigs = new TalonFXConfiguration();

		// 1 Rotation = 1 Rotation
		motorConfigs.Feedback.SensorToMechanismRatio = 1.0;

		// Set slot 0 configs
		var slot0Configs = motorConfigs.Slot0;
		slot0Configs.kS = RobotConstants.Elevator.KS; // Static friction compensation (v/rot)

		slot0Configs.kV = RobotConstants.Elevator.KV; // Velocity feed forward (v/rot)
		slot0Configs.kA = RobotConstants.Elevator.KA; // Acceleration feed forward (v/rot)

		slot0Configs.kP = RobotConstants.Elevator.KP; // Position error gain (p amount/%v error)
		slot0Configs.kI = RobotConstants.Elevator.KI; // Integral gain for steady-state error
		slot0Configs.kD = RobotConstants.Elevator.KD; // Derivative gain for damping

		slot0Configs.kG = RobotConstants.Elevator.KG; // Gravity compensation (v/rot)
		slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

		// Set motion magic
		var motionMagicConfigs = motorConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = RobotConstants.Elevator.CRUISE_VELOCITY;
		motionMagicConfigs.MotionMagicAcceleration = RobotConstants.Elevator.ACCELERATION;
		motionMagicConfigs.MotionMagicJerk = RobotConstants.Elevator.JERK;
		motionMagicConfigs.MotionMagicExpo_kA = RobotConstants.Elevator.KA;
		motionMagicConfigs.MotionMagicExpo_kV = RobotConstants.Elevator.KV;

		// Apply soft limits
		motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
				RobotConstants.Elevator.MAX_HEIGHT / RobotConstants.Elevator.METERS_PER_MOTOR_ROTATION;
		motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RobotConstants.Elevator.MIN_HEIGHT;

		// Reset encoder to zero
		leftMotor_10.setPosition(0);
		rightMotor_9.setPosition(0);

		// Setup both motors
		setupMotors(motorConfigs);

		// Create motor request at default position
		magicMotion = new MotionMagicExpoVoltage(0).withSlot(0);
	}

	@Override
	public void updateInputs(ElevatorInputs inputs) {

		inputs.heightM =
				leftMotor_10.getPosition().getValueAsDouble()
						* RobotConstants.Elevator.METERS_PER_MOTOR_ROTATION;

		inputs.rotations = leftMotor_10.getPosition().getValueAsDouble();

		inputs.velocityMPS =
				leftMotor_10.getVelocity().getValueAsDouble()
						* RobotConstants.Elevator.METERS_PER_MOTOR_ROTATION;
		inputs.accelerationMPS2 =
				leftMotor_10.getAcceleration().getValueAsDouble()
						* RobotConstants.Elevator.METERS_PER_MOTOR_ROTATION;

		inputs.setpointM = magicMotion.Position * RobotConstants.Elevator.METERS_PER_MOTOR_ROTATION;
		inputs.leftMotorVoltage = leftMotor_10.getMotorVoltage().getValueAsDouble();
		inputs.rightMotorVoltage = rightMotor_9.getMotorVoltage().getValueAsDouble();
		inputs.leftMotorCurrent = leftMotor_10.getSupplyCurrent().getValueAsDouble();
		inputs.rightMotorCurrent = rightMotor_9.getSupplyCurrent().getValueAsDouble();
	}

	@Override
	public void setPositionM(double newPositionM) {

		double targetRot = newPositionM / RobotConstants.Elevator.METERS_PER_MOTOR_ROTATION;

		// Update motor request
		magicMotion.withPosition(targetRot);

		leftMotor_10.setControl(magicMotion);
		rightMotor_9.setControl(magicMotion);
	}

	@Override
	public void setVoltage(double voltage) {
		leftMotor_10.setVoltage(voltage);
		rightMotor_9.setVoltage(voltage);
	}

	public void setupMotors(TalonFXConfiguration motorConfigs) {
		// Needs CCW+ to bring elevator up
		var rightMotorConfig = motorConfigs;
		rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		var rightMotorConfigStatus = rightMotor_9.getConfigurator().apply(motorConfigs);

		// Check if the configuration was successful
		if (rightMotorConfigStatus != StatusCode.OK) {
			DriverStation.reportWarning(
					"Failed to apply right motor configuration: " + rightMotorConfigStatus, false);
		}

		// needs CW+ to bring elevator up
		var leftMotorConfig = motorConfigs;
		leftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		var leftMotorConfigStatus = leftMotor_10.getConfigurator().apply(motorConfigs);

		// Check if the configuration was successful
		if (leftMotorConfigStatus != StatusCode.OK) {
			DriverStation.reportWarning(
					"Failed to apply left motor configuration: " + leftMotorConfigStatus, false);
		}

		// Set brake mode
		leftMotor_10.setNeutralMode(NeutralModeValue.Brake);
		rightMotor_9.setNeutralMode(NeutralModeValue.Brake);
	}
}
