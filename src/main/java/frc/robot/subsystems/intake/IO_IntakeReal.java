// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.RobotConstants;
import frc.robot.util.IRBeamBreak;

public class IO_IntakeReal implements IO_IntakeBase {

	private SparkMax armMotor;
	private SparkMax wheelMotor;
	private IRBeamBreak sensor;

	public IO_IntakeReal() {

		armMotor = new SparkMax(RobotConstants.Intake.ARM_MOTOR_ID, MotorType.kBrushless);
		wheelMotor = new SparkMax(RobotConstants.Intake.WHEEL_MOTOR_ID, MotorType.kBrushless);
		sensor = new IRBeamBreak(RobotConstants.Intake.SENSOR_RIO_ID);

		SparkMaxConfig wheelSparkMaxConfig = new SparkMaxConfig();
		wheelSparkMaxConfig.smartCurrentLimit(RobotConstants.Intake.WHEEL_MOTOR_CURRENT_LIMIT);
		wheelMotor.configure(
				wheelSparkMaxConfig,
				SparkBase.ResetMode.kNoResetSafeParameters,
				SparkBase.PersistMode.kPersistParameters);

		SparkMaxConfig armSparkMaxConfig = new SparkMaxConfig();
		armSparkMaxConfig.absoluteEncoder.positionConversionFactor(
				RobotConstants.Intake.ARM_ENCODER_FACTOR);
		armSparkMaxConfig.absoluteEncoder.inverted(true);
		armSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
		armSparkMaxConfig.inverted(true);

		ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();

		closedLoopConfig.p(RobotConstants.Intake.ARM_P);
		closedLoopConfig.i(RobotConstants.Intake.ARM_I);
		closedLoopConfig.d(RobotConstants.Intake.ARM_D);

		armSparkMaxConfig.apply(closedLoopConfig);

		armMotor.configure(
				armSparkMaxConfig,
				SparkBase.ResetMode.kNoResetSafeParameters,
				SparkBase.PersistMode.kPersistParameters);

		armMotor.getEncoder().setPosition(0);
	}

	@Override
	public void updateInputs(IntakeInputs inputs) {

		inputs.armAngleDegrees =
				armMotor.getAbsoluteEncoder().getPosition() + RobotConstants.Intake.ARM_ENCODER_LOOP_OFFSET;
		// inputs.armMotorVoltage = armMotor.getAppliedOutput();
		inputs.armMotorCurrent = armMotor.getOutputCurrent();
		inputs.wheelMotorCurrent = wheelMotor.getOutputCurrent();
		inputs.wheelRPM = wheelMotor.getEncoder().getVelocity();
		inputs.sensor = sensor.getState();
	}

	@Override
	public void setArmAngle(double angle) {
		armMotor
				.getClosedLoopController()
				.setReference(angle + RobotConstants.Intake.ARM_ENCODER_PID_OFFSET, ControlType.kPosition);
	}

	@Override
	public void setIntakeSpeed(double speed) {
		wheelMotor.set(speed);
	}
}
