// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IO_ElevatorBase {

	@AutoLog
	public static class ElevatorInputs implements LoggableInputs {

		public double heightMM = 0.0;
		public double velocityMMPS = 0.0;
		public double accelerationMMPS2 = 0.0;
		public double setpointMM = 0.0;

		public double leftMotorVoltage = 0.0;
		public double rightMotorVoltage = 0.0;

		public double leftMotorCurrent = 0.0;
		public double rightMotorCurrent = 0.0;

		public double leftMotorPower = 0.0;
		public double rightMotorPower = 0.0;

		@Override
		public void toLog(LogTable table) {
			table.put("heightMM", heightMM);
			table.put("velocityMMPS", velocityMMPS);
			table.put("accelerationMMPS2", accelerationMMPS2);
			table.put("setpointMM", setpointMM);
			table.put("leftMotorVoltage", leftMotorVoltage);
			table.put("rightMotorVoltage", rightMotorVoltage);
			table.put("leftMotorCurrent", leftMotorCurrent);
			table.put("rightMotorCurrent", rightMotorCurrent);
			table.put("leftMotorPower", leftMotorPower);
			table.put("rightMotorPower", rightMotorPower);
		}

		@Override
		public void fromLog(LogTable table) {
			heightMM = table.get("heightMM", heightMM);
			velocityMMPS = table.get("velocityMMPS", velocityMMPS);
			accelerationMMPS2 = table.get("accelerationMMPS2", accelerationMMPS2);
			setpointMM = table.get("setpointMM", setpointMM);
			leftMotorVoltage = table.get("leftMotorVoltage", leftMotorVoltage);
			rightMotorVoltage = table.get("rightMotorVoltage", rightMotorVoltage);
			leftMotorCurrent = table.get("leftMotorCurrent", leftMotorCurrent);
			rightMotorCurrent = table.get("rightMotorCurrent", rightMotorCurrent);
			leftMotorPower = table.get("leftMotorPower", leftMotorPower);
			rightMotorPower = table.get("rightMotorPower", rightMotorPower);
		}
	}

	/** Updates the set of loggable inputs. */
	public void updateInputs(ElevatorInputs inputs);

	public void setVoltage(double voltage);

	public void setPositionM(double newPositionM);
}
