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

/**
 * This class manages the selection of autonomous routines for the robot.
 * It loads available autonomous routines, allows for selection, and provides
 * access to the selected routine.
 */
public class AutoSelector {
    private final SendableChooser<Command> autoChooser;
    private final Map<String, Command> autos;
    private String defaultAutoName;

    /**
     * Constructs an AutoSelector, initializing the autonomous routine chooser
     * and loading available routines.
     */
    public AutoSelector() {
        this.autoChooser = new SendableChooser<>();
        this.autos = new HashMap<>();
        this.defaultAutoName = "";

        loadAutos();
        setDefaultAuto();
        putChooserToSmartDashboard();
    }

    /**
     * Loads all available autonomous routines from the AutoBuilder.
     */
    private void loadAutos() {
        List<String> autoNames = AutoBuilder.getAllAutoNames();
        for (String autoName : autoNames) {
            Command autoCommand = AutoBuilder.buildAuto(autoName);
            autos.put(autoName, autoCommand);
            autoChooser.addOption(autoName, autoCommand);
        }
    }

    /**
     * Sets the first loaded autonomous routine as the default if any are available.
     */
    private void setDefaultAuto() {
        if (!autos.isEmpty()) {
            String firstAutoName = autos.keySet().iterator().next();
            selectAuto(firstAutoName);
        }
    }

    /**
     * Adds the autonomous routine chooser to the SmartDashboard and logs available options.
     */
    private void putChooserToSmartDashboard() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
        System.out.println("Auto Chooser put to SmartDashboard");

        // Print out the available options
        for (String autoName : autos.keySet()) {
            System.out.println("Available auto: " + autoName);
        }
    }

    /**
     * Selects an autonomous routine as the default option.
     * @param autoName The name of the autonomous routine to select.
     */
    public void selectAuto(String autoName) {
        if (autos.containsKey(autoName)) {
            defaultAutoName = autoName;
            autoChooser.setDefaultOption(autoName, autos.get(autoName));
        } else {
            System.out.println(
                    "Warning: Auto '" + autoName + "' does not exist. Default auto unchanged.");
        }
    }

    /**
     * @return A copy of the map of all available autonomous routines.
     */
    public Map<String, Command> getAutos() {
        return new HashMap<>(autos);
    }

    /**
     * @return The SendableChooser for autonomous routines.
     */
    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }

    /**
     * @return The name of the default autonomous routine.
     */
    public String getDefaultAutoName() {
        return defaultAutoName;
    }

    /**
     * @return The currently selected autonomous Command.
     */
    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}