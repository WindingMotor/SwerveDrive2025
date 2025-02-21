// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IO_ElevatorBase {

	@AutoLog
	public static class ElevatorInputs implements LoggableInputs {

		public double heightM = 0.0;
		public double rotations = 0.0;
		public double velocityMPS = 0.0;
		public double accelerationMPS2 = 0.0;
		public double setpointM = 0.0;

		public double leftMotorVoltage = 0.0;
		public double rightMotorVoltage = 0.0;

		public double leftMotorCurrent = 0.0;
		public double rightMotorCurrent = 0.0;

		@Override
		public void toLog(LogTable table) {
			table.put("heightM", heightM);
			table.put("rotations", rotations);
			table.put("velocityMPS", velocityMPS);
			table.put("accelerationMPS2", accelerationMPS2);
			table.put("setpointM", setpointM);
			table.put("leftMotorVoltage", leftMotorVoltage);
			table.put("rightMotorVoltage", rightMotorVoltage);
			table.put("leftMotorCurrent", leftMotorCurrent);
			table.put("rightMotorCurrent", rightMotorCurrent);
		}

		@Override
		public void fromLog(LogTable table) {
			heightM = table.get("heightM", heightM);
			rotations = table.get("rotations", rotations);
			velocityMPS = table.get("velocityMPS", velocityMPS);
			accelerationMPS2 = table.get("accelerationMPS2", accelerationMPS2);
			setpointM = table.get("setpointM", setpointM);
			leftMotorVoltage = table.get("leftMotorVoltage", leftMotorVoltage);
			rightMotorVoltage = table.get("rightMotorVoltage", rightMotorVoltage);
			leftMotorCurrent = table.get("leftMotorCurrent", leftMotorCurrent);
			rightMotorCurrent = table.get("rightMotorCurrent", rightMotorCurrent);
		}
	}

	/** Updates the set of loggable inputs. */
	public void updateInputs(ElevatorInputs inputs);

	public void setVoltage(double voltage);

	public void setPositionM(double newPositionM);
}
