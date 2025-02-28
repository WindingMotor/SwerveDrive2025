// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IO_ClimbBase {

	@AutoLog
	public static class ClimbInputs implements LoggableInputs {

		public double motorCurrent = 0.0;
		public double motorPosition = 0.0;
		public double motorVelocity = 0.0;

		@Override
		public void toLog(LogTable table) {
			table.put("MotorCurrent", motorCurrent);
			table.put("MotorPosition", motorPosition);
			table.put("MotorVelocity", motorVelocity);
		}

		@Override
		public void fromLog(LogTable table) {
			motorCurrent = table.get("MotorCurrent", motorCurrent);
			motorPosition = table.get("MotorPosition", motorPosition);
			motorVelocity = table.get("MotorVelocity", motorVelocity);
		}
	}

	/** Updates the set of loggable inputs. */
	public void updateInputs(ClimbInputs inputs);

	public void setMotorSpeed(double speed);
}
