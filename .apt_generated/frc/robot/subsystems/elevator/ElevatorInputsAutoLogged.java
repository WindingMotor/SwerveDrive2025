// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorInputsAutoLogged extends IO_ElevatorBase.ElevatorInputs
		implements LoggableInputs, Cloneable {
	@Override
	public void toLog(LogTable table) {
		table.put("HeightM", heightM);
		table.put("Rotations", rotations);
		table.put("VelocityMPS", velocityMPS);
		table.put("AccelerationMPS2", accelerationMPS2);
		table.put("SetpointM", setpointM);
		table.put("LeftMotorVoltage", leftMotorVoltage);
		table.put("RightMotorVoltage", rightMotorVoltage);
		table.put("LeftMotorCurrent", leftMotorCurrent);
		table.put("RightMotorCurrent", rightMotorCurrent);
	}

	@Override
	public void fromLog(LogTable table) {
		heightM = table.get("HeightM", heightM);
		rotations = table.get("Rotations", rotations);
		velocityMPS = table.get("VelocityMPS", velocityMPS);
		accelerationMPS2 = table.get("AccelerationMPS2", accelerationMPS2);
		setpointM = table.get("SetpointM", setpointM);
		leftMotorVoltage = table.get("LeftMotorVoltage", leftMotorVoltage);
		rightMotorVoltage = table.get("RightMotorVoltage", rightMotorVoltage);
		leftMotorCurrent = table.get("LeftMotorCurrent", leftMotorCurrent);
		rightMotorCurrent = table.get("RightMotorCurrent", rightMotorCurrent);
	}

	public ElevatorInputsAutoLogged clone() {
		ElevatorInputsAutoLogged copy = new ElevatorInputsAutoLogged();
		copy.heightM = this.heightM;
		copy.rotations = this.rotations;
		copy.velocityMPS = this.velocityMPS;
		copy.accelerationMPS2 = this.accelerationMPS2;
		copy.setpointM = this.setpointM;
		copy.leftMotorVoltage = this.leftMotorVoltage;
		copy.rightMotorVoltage = this.rightMotorVoltage;
		copy.leftMotorCurrent = this.leftMotorCurrent;
		copy.rightMotorCurrent = this.rightMotorCurrent;
		return copy;
	}
}
