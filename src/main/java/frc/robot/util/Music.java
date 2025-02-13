// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;

public class Music {
	private Orchestra orchestra;
	private TalonFX[] fxes = {
		new TalonFX(1, "canivore"),
		new TalonFX(3, "canivore"),
		new TalonFX(5, "canivore"),
		new TalonFX(7, "canivore"),
	};

	public Music() {
		orchestra = new Orchestra();
		// Configure and add each motor
		for (TalonFX fx : fxes) {
			orchestra.addInstrument(fx);
		}
	}

	public void playSong(String songName) {
		var status = orchestra.loadMusic("music/" + songName);
		if (!status.isOK()) {
			DriverStation.reportError("Failed to load music file: " + songName, false);
			return;
		}
		orchestra.play();
	}

	public void stop() {
		if (orchestra.isPlaying()) {
			orchestra.stop();
		}
	}
}
