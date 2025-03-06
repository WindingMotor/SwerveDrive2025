// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeInputsAutoLogged extends IO_IntakeBase.IntakeInputs
		implements LoggableInputs, Cloneable {
	@Override
	public void toLog(LogTable table) {
		table.put("ArmAngleDegrees", armAngleDegrees);
		table.put("ArmMotorCurrent", armMotorCurrent);
		table.put("WheelMotorCurrent", wheelMotorCurrent);
		table.put("WheelRPM", wheelRPM);
		table.put("Sensor", sensor);
	}

	@Override
	public void fromLog(LogTable table) {
		armAngleDegrees = table.get("ArmAngleDegrees", armAngleDegrees);
		armMotorCurrent = table.get("ArmMotorCurrent", armMotorCurrent);
		wheelMotorCurrent = table.get("WheelMotorCurrent", wheelMotorCurrent);
		wheelRPM = table.get("WheelRPM", wheelRPM);
		sensor = table.get("Sensor", sensor);
	}

	public IntakeInputsAutoLogged clone() {
		IntakeInputsAutoLogged copy = new IntakeInputsAutoLogged();
		copy.armAngleDegrees = this.armAngleDegrees;
		copy.armMotorCurrent = this.armMotorCurrent;
		copy.wheelMotorCurrent = this.wheelMotorCurrent;
		copy.wheelRPM = this.wheelRPM;
		copy.sensor = this.sensor;
		return copy;
	}
}
