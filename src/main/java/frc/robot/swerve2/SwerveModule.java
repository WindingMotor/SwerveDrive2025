// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve2;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
	private final TalonFX driveMotor;
	private final TalonFX turningMotor;
	private final DutyCycleEncoder absoluteEncoder;

	// PID and feedforward constants
	private static final double kDriveP = 0.1;
	private static final double kDriveI = 0.0;
	private static final double kDriveD = 0.0;

	private static final double kTurningP = 2;
	private static final double kTurningI = 0.0;
	private static final double kTurningD = 0.0;

	private static final double kDriveGearRatio = 6.75; // L2 gear ratio
	private static final double kTurningGearRatio = 21.4285714286;
	private static final double kWheelDiameterMeters = 0.1016; // 4 inch wheels

	private final double absoluteEncoderOffsetDeg;
	private final PositionVoltage turnRequest;
	private final VelocityVoltage driveRequest;

	// Track setpoints for logging
	private double setpointAngleDegrees = 0.0;
	private double setpointVelocity = 0.0;

	private String name;

	public SwerveModule(
			int driveMotorId,
			int turningMotorId,
			int absoluteEncoderId,
			double absoluteEncoderOffset,
			String canBus,
			String name) {

		this.name = name;
		this.absoluteEncoderOffsetDeg = absoluteEncoderOffset;

		// Initialize the DutyCycleEncoder for the SRX Mag Encoder
		this.absoluteEncoder =
				new DutyCycleEncoder(
						absoluteEncoderId,
						360.0, // Full rotation in degrees
						0.0 // Zero position
						);

		driveMotor = new TalonFX(driveMotorId, canBus);
		turningMotor = new TalonFX(turningMotorId, canBus);

		// Configure drive motor
		var driveConfigs = new TalonFXConfiguration();
		driveConfigs.Feedback.SensorToMechanismRatio = kDriveGearRatio;

		var driveSlot0 = driveConfigs.Slot0;
		driveSlot0.kP = kDriveP;
		driveSlot0.kI = kDriveI;
		driveSlot0.kD = kDriveD;

		driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		driveConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		var driveStatus = driveMotor.getConfigurator().apply(driveConfigs);
		if (driveStatus != StatusCode.OK) {
			DriverStation.reportWarning(
					"Failed to apply drive motor configuration: " + driveStatus, false);
		}

		// Configure turning motor
		var turningConfigs = new TalonFXConfiguration();
		turningConfigs.Feedback.SensorToMechanismRatio = kTurningGearRatio;

		var turningSlot0 = turningConfigs.Slot0;
		turningSlot0.kP = kTurningP;
		turningSlot0.kI = kTurningI;
		turningSlot0.kD = kTurningD;

		turningConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		turningConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		var turningStatus = turningMotor.getConfigurator().apply(turningConfigs);
		if (turningStatus != StatusCode.OK) {
			DriverStation.reportWarning(
					"Failed to apply turning motor configuration: " + turningStatus, false);
		}

		// Create control requests
		turnRequest = new PositionVoltage(0).withSlot(0);
		driveRequest = new VelocityVoltage(0).withSlot(0);

		resetEncoders();
	}

	public double getDrivePosition() {
		return driveMotor.getPosition().getValueAsDouble()
				* kWheelDiameterMeters
				* Math.PI
				/ kDriveGearRatio;
	}

	public double getTurningPosition() {
		return turningMotor.getPosition().getValueAsDouble() * 360.0 / kTurningGearRatio;
	}

	public double getDriveVelocity() {
		return driveMotor.getVelocity().getValueAsDouble()
				* kWheelDiameterMeters
				* Math.PI
				/ kDriveGearRatio;
	}

	public double getAbsoluteEncoderDeg() {
		double angle = absoluteEncoder.get();
		angle -= absoluteEncoderOffsetDeg;
		// Normalize angle to be between -180 and 180
		while (angle > 180) angle -= 360;
		while (angle < -180) angle += 360;
		return angle;
	}

	public void resetEncoders() {
		driveMotor.setPosition(0);
		turningMotor.setPosition(getAbsoluteEncoderDeg() * kTurningGearRatio / 360.0);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningPosition()));
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
				getDrivePosition(), Rotation2d.fromDegrees(getTurningPosition()));
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		// Optimize the reference state to avoid spinning further than 90 degrees
		SwerveModuleState optimizedState =
				SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getTurningPosition()));

		// Store setpoints for logging
		setpointVelocity = optimizedState.speedMetersPerSecond;
		setpointAngleDegrees = optimizedState.angle.getDegrees();

		driveMotor.setControl(
				driveRequest.withVelocity(
						optimizedState.speedMetersPerSecond
								* kDriveGearRatio
								/ (kWheelDiameterMeters * Math.PI)));

		turningMotor.setControl(
				turnRequest.withPosition(optimizedState.angle.getDegrees() * kTurningGearRatio / 360.0));
	}

	public void stop() {
		driveMotor.stopMotor();
		turningMotor.stopMotor();
	}

	public void log() {
		// Module identification and basic state
		SmartDashboard.putString(name + "/Module Name", name);

		// Absolute encoder readings
		SmartDashboard.putNumber(name + "/Absolute Encoder Raw", absoluteEncoder.get());
		SmartDashboard.putNumber(name + "/Absolute Encoder With Offset (deg)", getAbsoluteEncoderDeg());
		SmartDashboard.putNumber(name + "/Absolute Encoder Offset (deg)", absoluteEncoderOffsetDeg);

		// Drive motor telemetry
		SmartDashboard.putNumber(name + "/Drive Position (m)", getDrivePosition());
		SmartDashboard.putNumber(name + "/Drive Velocity (m/s)", getDriveVelocity());
		SmartDashboard.putNumber(
				name + "/Drive Motor Raw Position", driveMotor.getPosition().getValueAsDouble());
		SmartDashboard.putNumber(
				name + "/Drive Motor Raw Velocity", driveMotor.getVelocity().getValueAsDouble());

		// Turning motor telemetry
		SmartDashboard.putNumber(name + "/Turn Angle (deg)", getTurningPosition());
		SmartDashboard.putNumber(
				name + "/Turn Motor Raw Position", turningMotor.getPosition().getValueAsDouble());

		// Current state
		SwerveModuleState currentState = getState();
		SmartDashboard.putNumber(name + "/Current Speed (m/s)", currentState.speedMetersPerSecond);
		SmartDashboard.putNumber(name + "/Current Angle (deg)", currentState.angle.getDegrees());

		// Setpoints
		SmartDashboard.putNumber(name + "/Setpoint Speed (m/s)", setpointVelocity);
		SmartDashboard.putNumber(name + "/Setpoint Angle (deg)", setpointAngleDegrees);

		// Motor status
		SmartDashboard.putNumber(
				name + "/Drive Motor Temperature", driveMotor.getDeviceTemp().getValueAsDouble());
		SmartDashboard.putNumber(
				name + "/Turn Motor Temperature", turningMotor.getDeviceTemp().getValueAsDouble());
		SmartDashboard.putNumber(
				name + "/Drive Motor Current", driveMotor.getSupplyCurrent().getValueAsDouble());
		SmartDashboard.putNumber(
				name + "/Turn Motor Current", turningMotor.getSupplyCurrent().getValueAsDouble());
	}
}
