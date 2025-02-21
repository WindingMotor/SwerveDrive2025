// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.TunerConstants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class IO_ModuleReal implements IO_ModuleBase {
	private final SwerveModuleConstants<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			constants;

	// Hardware objects
	private final TalonFX driveTalon;
	private final TalonFX turnTalon;

	private final Canandmag canandmag;

	// Voltage control requests
	private final VoltageOut voltageRequest = new VoltageOut(0);
	private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
	private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

	// Torque-current control requests
	private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
	private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
			new PositionTorqueCurrentFOC(0.0);
	private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
			new VelocityTorqueCurrentFOC(0.0);

	// Timestamp inputs from Phoenix thread
	private final Queue<Double> timestampQueue;

	// Inputs from drive motor
	private final StatusSignal<Angle> drivePosition;
	private final Queue<Double> drivePositionQueue;
	private final StatusSignal<AngularVelocity> driveVelocity;
	private final StatusSignal<Voltage> driveAppliedVolts;
	private final StatusSignal<Current> driveCurrent;

	// Inputs from turn motor
	private final StatusSignal<Angle> turnPosition;
	private final Queue<Double> turnPositionQueue;
	private final StatusSignal<AngularVelocity> turnVelocity;
	private final StatusSignal<Voltage> turnAppliedVolts;
	private final StatusSignal<Current> turnCurrent;

	// Connection debouncers
	private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
	private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
	private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

	public IO_ModuleReal(
			SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
					constants) {
		this.constants = constants;
		driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
		turnTalon = new TalonFX(constants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
		canandmag = new Canandmag(constants.EncoderId);

		// Configure drive motor
		var driveConfig = constants.DriveMotorInitialConfigs;
		driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		driveConfig.Slot0 = constants.DriveMotorGains;
		driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
		driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
		driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
		driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveConfig.MotorOutput.Inverted =
				constants.DriveMotorInverted
						? InvertedValue.Clockwise_Positive
						: InvertedValue.CounterClockwise_Positive;
		tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
		tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

		// Configure turn motor
		var turnConfig = new TalonFXConfiguration();
		turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		turnConfig.Slot0 = constants.SteerMotorGains;
		turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		turnConfig.Feedback.SensorToMechanismRatio = 1;
		turnConfig.Feedback.SensorToMechanismRatio = TunerConstants.kSteerGearRatio;
		turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
		turnConfig.MotionMagic.MotionMagicAcceleration =
				turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
		turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
		turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
		turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
		turnConfig.MotorOutput.Inverted =
				constants.SteerMotorInverted
						? InvertedValue.Clockwise_Positive
						: InvertedValue.CounterClockwise_Positive;
		tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

		// Configure Canandmag
		CanandmagSettings canandmagSettings = new CanandmagSettings();
		canandmagSettings.setZeroOffset(constants.EncoderOffset);
		canandmagSettings.setInvertDirection(true);
		canandmag.setSettings(canandmagSettings);

		// Verify Canandmag is connected
		if (!canandmag.isConnected()) {
			throw new RuntimeException("Canandmag not connected during initialization");
		}

		// Get absolute position (0 to 1) and verify it's valid
		double absolutePosition = canandmag.getAbsPosition();

		// No need to convert units because TalonFX.setPosition() expects rotations in Phoenix v6
		// The Canandmag returns 0-1 which matches the rotations unit expected by the TalonFX
		tryUntilOk(10, () -> turnTalon.setPosition(absolutePosition, 0.25));

		// Create timestamp queue
		timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

		// Create drive status signals
		drivePosition = driveTalon.getPosition();
		drivePositionQueue =
				PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
		driveVelocity = driveTalon.getVelocity();
		driveAppliedVolts = driveTalon.getMotorVoltage();
		driveCurrent = driveTalon.getStatorCurrent();

		// Create turn status signals
		turnPosition = turnTalon.getPosition();
		turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition());
		turnVelocity = turnTalon.getVelocity();
		turnAppliedVolts = turnTalon.getMotorVoltage();
		turnCurrent = turnTalon.getStatorCurrent();

		// Configure periodic frames
		BaseStatusSignal.setUpdateFrequencyForAll(
				Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
		BaseStatusSignal.setUpdateFrequencyForAll(
				50.0,
				driveVelocity,
				driveAppliedVolts,
				driveCurrent,
				turnVelocity,
				turnAppliedVolts,
				turnCurrent);
		ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
	}

	@Override
	public void updateInputs(ModuleInputs inputs) {
		// Refresh all signals
		var driveStatus =
				BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
		var turnStatus =
				BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);

		// Update drive inputs
		inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
		inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
		inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
		inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
		inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

		// Update turn inputs
		inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
		inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(canandmag.isConnected());
		inputs.turnAbsolutePosition = Rotation2d.fromRotations(canandmag.getAbsPosition());
		inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
		inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
		inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
		inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

		// Update odometry inputs
		inputs.odometryTimestamps =
				timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
		inputs.odometryDrivePositionsRad =
				drivePositionQueue.stream()
						.mapToDouble((Double value) -> Units.rotationsToRadians(value))
						.toArray();
		inputs.odometryTurnPositions =
				turnPositionQueue.stream()
						.map((Double value) -> Rotation2d.fromRotations(value))
						.toArray(Rotation2d[]::new);
		timestampQueue.clear();
		drivePositionQueue.clear();
		turnPositionQueue.clear();
	}

	@Override
	public void setDriveOpenLoop(double output) {
		driveTalon.setControl(
				switch (constants.DriveMotorClosedLoopOutput) {
					case Voltage -> voltageRequest.withOutput(output);
					case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
				});
	}

	@Override
	public void setTurnOpenLoop(double output) {
		turnTalon.setControl(
				switch (constants.SteerMotorClosedLoopOutput) {
					case Voltage -> voltageRequest.withOutput(output);
					case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
				});
	}

	@Override
	public void setDriveVelocity(double velocityRadPerSec) {
		double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
		driveTalon.setControl(
				switch (constants.DriveMotorClosedLoopOutput) {
					case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
					case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
				});
	}

	@Override
	public void setTurnPosition(Rotation2d rotation) {
		turnTalon.setControl(
				switch (constants.SteerMotorClosedLoopOutput) {
					case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
					case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(
							rotation.getRotations());
				});
	}
}
