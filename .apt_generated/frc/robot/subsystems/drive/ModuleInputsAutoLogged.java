// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleInputsAutoLogged extends IO_ModuleBase.ModuleInputs
		implements LoggableInputs, Cloneable {
	@Override
	public void toLog(LogTable table) {
		table.put("DriveConnected", driveConnected);
		table.put("DrivePositionRad", drivePositionRad);
		table.put("DriveVelocityRadPerSec", driveVelocityRadPerSec);
		table.put("DriveAppliedVolts", driveAppliedVolts);
		table.put("DriveCurrentAmps", driveCurrentAmps);
		table.put("TurnConnected", turnConnected);
		table.put("TurnEncoderConnected", turnEncoderConnected);
		table.put("TurnAbsolutePosition", turnAbsolutePosition);
		table.put("TurnPosition", turnPosition);
		table.put("TurnVelocityRadPerSec", turnVelocityRadPerSec);
		table.put("TurnAppliedVolts", turnAppliedVolts);
		table.put("TurnCurrentAmps", turnCurrentAmps);
		table.put("OdometryTimestamps", odometryTimestamps);
		table.put("OdometryDrivePositionsRad", odometryDrivePositionsRad);
		table.put("OdometryTurnPositions", odometryTurnPositions);
	}

	@Override
	public void fromLog(LogTable table) {
		driveConnected = table.get("DriveConnected", driveConnected);
		drivePositionRad = table.get("DrivePositionRad", drivePositionRad);
		driveVelocityRadPerSec = table.get("DriveVelocityRadPerSec", driveVelocityRadPerSec);
		driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
		driveCurrentAmps = table.get("DriveCurrentAmps", driveCurrentAmps);
		turnConnected = table.get("TurnConnected", turnConnected);
		turnEncoderConnected = table.get("TurnEncoderConnected", turnEncoderConnected);
		turnAbsolutePosition = table.get("TurnAbsolutePosition", turnAbsolutePosition);
		turnPosition = table.get("TurnPosition", turnPosition);
		turnVelocityRadPerSec = table.get("TurnVelocityRadPerSec", turnVelocityRadPerSec);
		turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts);
		turnCurrentAmps = table.get("TurnCurrentAmps", turnCurrentAmps);
		odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
		odometryDrivePositionsRad = table.get("OdometryDrivePositionsRad", odometryDrivePositionsRad);
		odometryTurnPositions = table.get("OdometryTurnPositions", odometryTurnPositions);
	}

	public ModuleInputsAutoLogged clone() {
		ModuleInputsAutoLogged copy = new ModuleInputsAutoLogged();
		copy.driveConnected = this.driveConnected;
		copy.drivePositionRad = this.drivePositionRad;
		copy.driveVelocityRadPerSec = this.driveVelocityRadPerSec;
		copy.driveAppliedVolts = this.driveAppliedVolts;
		copy.driveCurrentAmps = this.driveCurrentAmps;
		copy.turnConnected = this.turnConnected;
		copy.turnEncoderConnected = this.turnEncoderConnected;
		copy.turnAbsolutePosition = this.turnAbsolutePosition;
		copy.turnPosition = this.turnPosition;
		copy.turnVelocityRadPerSec = this.turnVelocityRadPerSec;
		copy.turnAppliedVolts = this.turnAppliedVolts;
		copy.turnCurrentAmps = this.turnCurrentAmps;
		copy.odometryTimestamps = this.odometryTimestamps.clone();
		copy.odometryDrivePositionsRad = this.odometryDrivePositionsRad.clone();
		copy.odometryTurnPositions = this.odometryTurnPositions.clone();
		return copy;
	}
}
