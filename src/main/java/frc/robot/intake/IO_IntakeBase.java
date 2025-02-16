// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IO_IntakeBase {

	@AutoLog
	public static class IntakeInputs implements LoggableInputs {

		public double armAngleDegrees = 0.0;
		public double armMotorCurrent = 0.0;
		public double wheelMotorCurrent = 0.0;
		public double wheelRPM = 0.0;
		public boolean sensor = false;

		@Override
		public void toLog(LogTable table) {
			table.put("ArmAngleDegrees", armAngleDegrees);
			table.put("ArmMotorCurrent", armMotorCurrent);
			table.put("wheelMotorCurrent", wheelMotorCurrent);
			table.put("wheelRPM", wheelRPM);
			table.put("Sensor", sensor);
		}

		@Override
		public void fromLog(LogTable table) {
			armAngleDegrees = table.get("ArmAngleDegrees", armAngleDegrees);
			armMotorCurrent = table.get("ArmMotorCurrent", armMotorCurrent);
			wheelMotorCurrent = table.get("wheelMotorCurrent", wheelMotorCurrent);
			wheelRPM = table.get("wheelRPM", wheelRPM);
			sensor = table.get("Sensor", sensor);
		}
	}

	/** Updates the set of loggable inputs. */
	public void updateInputs(IntakeInputs inputs);

	public void setArmAngle(double angle);

	public void setIntakeSpeed(double speed);
}
