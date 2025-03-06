// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimbInputsAutoLogged extends IO_ClimbBase.ClimbInputs
		implements LoggableInputs, Cloneable {
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

	public ClimbInputsAutoLogged clone() {
		ClimbInputsAutoLogged copy = new ClimbInputsAutoLogged();
		copy.motorCurrent = this.motorCurrent;
		copy.motorPosition = this.motorPosition;
		copy.motorVelocity = this.motorVelocity;
		return copy;
	}
}
