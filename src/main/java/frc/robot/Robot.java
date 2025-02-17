// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.RobotConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;

	private final RobotContainer m_robotContainer;

	@SuppressWarnings("resource")
	public Robot() {
		m_robotContainer = new RobotContainer();

		// Force the Redux server to start on port 7244 on the RoboRIO
		// To use download Redux Alchemist navigate to settings then enter RoboRIO IP:
		// roboRIO-2106-FRC.local
		// if (RobotConstants.FORCE_REDUX_SERVER_ON) {
		//	CanandEventLoop.getInstance();
		// }

		// Log build metadata
		Logger.recordMetadata("Maven Name", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("Build Date", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("ProjectName", "SwerveDrive2025");
		Logger.recordMetadata("Robot Mode", RobotConstants.ROBOT_MODE.toString());
		Logger.recordMetadata("Git Branch", BuildConstants.GIT_BRANCH);
		Logger.recordMetadata("Authors", "(WindingMotor) Isaac S - (PeskyBuzz) Rae");

		switch (RobotConstants.ROBOT_MODE) {
			case REAL:
				// Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
				Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
				new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
				break;
			case SIM:
				setUseTiming(false); // Run as fast as possible
				Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
				break;
			case REPLAY:
				// Replay Stuff
				String logPath =
						LogFileUtil
								.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
				Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log

				Logger.addDataReceiver(
						new WPILOGWriter(
								LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
				break;
		}

		Logger.start();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {

		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
