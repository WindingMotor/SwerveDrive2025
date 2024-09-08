// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveInputsAutoLogged extends IO_SwerveBase.SwerveInputs
		implements LoggableInputs, Cloneable {
	@Override
	public void toLog(LogTable table) {
		table.put("Pose", pose);
		table.put("Heading", heading);
		table.put("Velocity", velocity);
	}

	@Override
	public void fromLog(LogTable table) {
		pose = table.get("Pose", pose);
		heading = table.get("Heading", heading);
		velocity = table.get("Velocity", velocity);
	}

	public SwerveInputsAutoLogged clone() {
		SwerveInputsAutoLogged copy = new SwerveInputsAutoLogged();
		copy.pose = this.pose;
		copy.heading = this.heading;
		copy.velocity = this.velocity;
		return copy;
	}
}
