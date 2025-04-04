// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.RobotConstants;
import java.lang.reflect.Field;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
	private static final double loopOverrunWarningTimeout = 0.2;

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
		Logger.recordMetadata("Authors", "(WindingMotor) Isaac S - 2106 Junkyard DOgs");

		switch (RobotConstants.ROBOT_MODE) {
			case REAL:
				Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
				// Logger.addDataReceiver(new RLOGServer());
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

		// Adjust loop overrun warning timeout
		try {
			Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
			watchdogField.setAccessible(true);
			Watchdog watchdog = (Watchdog) watchdogField.get(this);
			watchdog.setTimeout(loopOverrunWarningTimeout);
		} catch (Exception e) {
			DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
		}

		// Log active commands
		/*
		Map<String, Integer> commandCounts = new HashMap<>();
		BiConsumer<Command, Boolean> logCommandFunction =
				(Command command, Boolean active) -> {
					String name = command.getName();
					int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
					commandCounts.put(name, count);
					Logger.recordOutput(
							"CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
					Logger.recordOutput("CommandsAll/" + name, count > 0);
				};
		CommandScheduler.getInstance()
				.onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
		CommandScheduler.getInstance()
				.onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
		CommandScheduler.getInstance()
				.onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

				*/

		// Configure brownout voltage
		// RobotController.setBrownoutVoltage(6.0);

		// Switch thread to high priority to improve loop timing
		// Threads.setCurrentThreadPriority(true, 10);
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
