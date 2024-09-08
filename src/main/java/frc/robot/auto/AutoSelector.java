// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AutoSelector {
	private final SendableChooser<Command> autoChooser;
	private final Map<String, Command> autos;
	private String defaultAutoName;

	public AutoSelector() {
		this.autoChooser = new SendableChooser<>();
		this.autos = new HashMap<>();
		this.defaultAutoName = "";

		loadAutos();
		setDefaultAuto();
		putChooserToSmartDashboard();
	}

	private void loadAutos() {
		List<String> autoNames = AutoBuilder.getAllAutoNames();
		for (String autoName : autoNames) {
			Command autoCommand = AutoBuilder.buildAuto(autoName);
			autos.put(autoName, autoCommand);
			autoChooser.addOption(autoName, autoCommand);
		}
	}

	private void setDefaultAuto() {
		if (!autos.isEmpty()) {
			String firstAutoName = autos.keySet().iterator().next();
			selectAuto(firstAutoName);
		}
	}

	private void putChooserToSmartDashboard() {
		SmartDashboard.putData("Auto Chooser", autoChooser);
		System.out.println("Auto Chooser put to SmartDashboard");

		// Print out the available options
		for (String autoName : autos.keySet()) {
			System.out.println("Available auto: " + autoName);
		}
	}

	public void selectAuto(String autoName) {
		if (autos.containsKey(autoName)) {
			defaultAutoName = autoName;
			autoChooser.setDefaultOption(autoName, autos.get(autoName));
		} else {
			System.out.println(
					"Warning: Auto '" + autoName + "' does not exist. Default auto unchanged.");
		}
	}

	public Map<String, Command> getAutos() {
		return new HashMap<>(autos);
	}

	public SendableChooser<Command> getAutoChooser() {
		return autoChooser;
	}

	public String getDefaultAutoName() {
		return defaultAutoName;
	}

	public Command getSelectedAuto() {
		return autoChooser.getSelected();
	}
}
