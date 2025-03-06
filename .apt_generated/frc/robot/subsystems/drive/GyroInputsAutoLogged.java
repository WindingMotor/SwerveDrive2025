// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroInputsAutoLogged extends IO_GyroBase.GyroInputs
		implements LoggableInputs, Cloneable {
	@Override
	public void toLog(LogTable table) {
		table.put("Connected", connected);
		table.put("YawPosition", yawPosition);
		table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
		table.put("OdometryYawTimestamps", odometryYawTimestamps);
		table.put("OdometryYawPositions", odometryYawPositions);
	}

	@Override
	public void fromLog(LogTable table) {
		connected = table.get("Connected", connected);
		yawPosition = table.get("YawPosition", yawPosition);
		yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
		odometryYawTimestamps = table.get("OdometryYawTimestamps", odometryYawTimestamps);
		odometryYawPositions = table.get("OdometryYawPositions", odometryYawPositions);
	}

	public GyroInputsAutoLogged clone() {
		GyroInputsAutoLogged copy = new GyroInputsAutoLogged();
		copy.connected = this.connected;
		copy.yawPosition = this.yawPosition;
		copy.yawVelocityRadPerSec = this.yawVelocityRadPerSec;
		copy.odometryYawTimestamps = this.odometryYawTimestamps.clone();
		copy.odometryYawPositions = this.odometryYawPositions.clone();
		return copy;
	}
}
