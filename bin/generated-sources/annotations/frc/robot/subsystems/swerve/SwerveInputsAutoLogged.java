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
		table.put("MaximumVelocity", maximumVelocity);
		table.put("MaximumAngularVelocity", maximumAngularVelocity);
		table.put("HeadingPID_P", headingPID_P);
		table.put("HeadingPID_I", headingPID_I);
		table.put("HeadingPID_D", headingPID_D);
		table.put("TranslationPID_P", translationPID_P);
		table.put("TranslationPID_I", translationPID_I);
		table.put("TranslationPID_D", translationPID_D);
		table.put("ConfigurationRadius", configurationRadius);
	}

	@Override
	public void fromLog(LogTable table) {
		pose = table.get("Pose", pose);
		heading = table.get("Heading", heading);
		velocity = table.get("Velocity", velocity);
		maximumVelocity = table.get("MaximumVelocity", maximumVelocity);
		maximumAngularVelocity = table.get("MaximumAngularVelocity", maximumAngularVelocity);
		headingPID_P = table.get("HeadingPID_P", headingPID_P);
		headingPID_I = table.get("HeadingPID_I", headingPID_I);
		headingPID_D = table.get("HeadingPID_D", headingPID_D);
		translationPID_P = table.get("TranslationPID_P", translationPID_P);
		translationPID_I = table.get("TranslationPID_I", translationPID_I);
		translationPID_D = table.get("TranslationPID_D", translationPID_D);
		configurationRadius = table.get("ConfigurationRadius", configurationRadius);
	}

	public SwerveInputsAutoLogged clone() {
		SwerveInputsAutoLogged copy = new SwerveInputsAutoLogged();
		copy.pose = this.pose;
		copy.heading = this.heading;
		copy.velocity = this.velocity;
		copy.maximumVelocity = this.maximumVelocity;
		copy.maximumAngularVelocity = this.maximumAngularVelocity;
		copy.headingPID_P = this.headingPID_P;
		copy.headingPID_I = this.headingPID_I;
		copy.headingPID_D = this.headingPID_D;
		copy.translationPID_P = this.translationPID_P;
		copy.translationPID_I = this.translationPID_I;
		copy.translationPID_D = this.translationPID_D;
		copy.configurationRadius = this.configurationRadius;
		return copy;
	}
}
