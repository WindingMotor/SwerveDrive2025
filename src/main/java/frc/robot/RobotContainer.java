// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.algae.CMD_ElevatorAlgae;
import frc.robot.commands.coral.CMD_ElevatorCoral;
import frc.robot.commands.drive.CMD_Drive;
import frc.robot.commands.generic.CMD_Eject;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.constants.InputConstants;
import frc.robot.elevator.IO_ElevatorReal;
import frc.robot.elevator.SUB_Elevator;
import frc.robot.intake.IO_IntakeReal;
import frc.robot.intake.SUB_Intake;
import frc.robot.superstructure.SUB_Superstructure;
import frc.robot.superstructure.SuperstructureState;
import frc.robot.swerve.IO_SwerveReal;
import frc.robot.swerve.SUB_Swerve;
import frc.robot.util.SUB_Led;
import frc.robot.vision.IO_VisionReal;
import frc.robot.vision.SUB_Vision;
import frc.robot.webserver.WebServer;
import java.io.File;

public class RobotContainer {
	// Controller Configuration
	private CommandXboxController driverController;
	private CommandXboxController operatorController;
	private InputConstants globalInputMap;

	// Subsystems
	private SUB_Swerve swerve;
	private SUB_Intake intake;
	private SUB_Vision vision;
	private SUB_Elevator elevator;
	private SUB_Superstructure superstructure;
	private SUB_Led led;
	private WebServer webServer;

	public RobotContainer() {
		// Initialize Controllers
		initializeControllers();

		// Initialize Subsystems
		initializeSubsystems();

		// Configure Robot Functionality
		configureDefaultCommands();
		configureWebserverCommands();
		configurePathPlannerCommands();
		configureButtonBindings();
	}

	private void initializeControllers() {
		driverController = new CommandXboxController(0);
		operatorController = new CommandXboxController(1);
		globalInputMap = InputConstants.TX16S_MAIN;
	}

	private void initializeSubsystems() {
		webServer = new WebServer();
		vision = new SUB_Vision(new IO_VisionReal());
		swerve =
				new SUB_Swerve(
						new IO_SwerveReal(new File(Filesystem.getDeployDirectory(), "swerve")), vision);
		intake = new SUB_Intake(new IO_IntakeReal());
		elevator = new SUB_Elevator(new IO_ElevatorReal());
		led = new SUB_Led();
		superstructure = new SUB_Superstructure(intake, elevator, led);
	}

	private void configureDefaultCommands() {
		swerve.setDefaultCommand(new CMD_Drive(swerve, driverController, globalInputMap));
	}

	private void configureWebserverCommands() {
		/*
		webServer.registerCommand("T1", swerve.driveToPose(FieldConstants.BLUE_TOP_TOP_LEFT));
		webServer.registerCommand("T2", swerve.driveToPose(FieldConstants.BLUE_TOP_TOP_RIGHT));
		webServer.registerCommand("TL1", swerve.driveToPose(FieldConstants.BLUE_TOP_LEFT_BOTTOM));
		webServer.registerCommand("TL2", swerve.driveToPose(FieldConstants.BLUE_TOP_LEFT_TOP));
		*/
	}

	private void configurePathPlannerCommands() {
		NamedCommands.registerCommand("Intake_Algae", new PrintCommand("Intake Algae"));
	}

	private void configureButtonBindings() {

		/*
		// State enum for tracking test sequence
		enum SysIdState {
			WAITING,
			DYN_FORWARD,
			DYN_REVERSE,
			QUASI_FORWARD,
			QUASI_REVERSE,
			COMPLETE
		}

		// Create atomic reference to track state
		AtomicReference<SysIdState> currentState = new AtomicReference<>(SysIdState.WAITING);

		// A button - Start logging and begin sequence
		operatorController
				.a()
				.onTrue(
						Commands.runOnce(
								() -> {
									StatusCode startStatus = SignalLogger.start();
									currentState.set(SysIdState.DYN_FORWARD);
									DriverStation.reportError(
											"SysId Started - Ready for Dynamic Forward Test", false);
								}));

		// X button - Run current test and advance to next
		operatorController
				.x()
				.onTrue(
						Commands.runOnce(
								() -> {
									elevator.setVoltage(0); // Stop any current movement

									switch (currentState.get()) {
										case DYN_FORWARD:
											elevator.sysIdDynamic(SysIdRoutine.Direction.kForward).schedule();
											currentState.set(SysIdState.DYN_REVERSE);
											DriverStation.reportError(
													"Dynamic Forward Complete - Ready for Dynamic Reverse", false);
											break;

										case DYN_REVERSE:
											elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule();
											currentState.set(SysIdState.QUASI_FORWARD);
											DriverStation.reportError(
													"Dynamic Reverse Complete - Ready for Quasistatic Forward", false);
											break;

										case QUASI_FORWARD:
											elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
											currentState.set(SysIdState.QUASI_REVERSE);
											DriverStation.reportError(
													"Quasistatic Forward Complete - Ready for Quasistatic Reverse", false);
											break;

										case QUASI_REVERSE:
											elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
											currentState.set(SysIdState.COMPLETE);
											DriverStation.reportError(
													"Quasistatic Reverse Complete - All Tests Done!", false);
											break;

										default:
											DriverStation.reportError("No test to run or sequence complete", false);
											break;
									}
								}));

		// B button - Stop logging and reset
		operatorController
				.b()
				.onTrue(
						Commands.runOnce(
								() -> {
									elevator.setVoltage(0);
									StatusCode stopStatus = SignalLogger.stop();
									currentState.set(SysIdState.WAITING);
									DriverStation.reportError(
											"SysId Stopped - Logger Status: " + stopStatus.toString(), false);
								}));

								*/

		// Extake

		operatorController.x().onTrue(new CMD_Eject(superstructure));

		// Coral Controls
		operatorController
				.rightBumper()
				.onTrue(new CMD_ElevatorCoral(superstructure, true)); // DPAD-UP - Coral up

		// Algae Controls
		operatorController.leftBumper().onTrue(new CMD_ElevatorAlgae(superstructure, true));

		// Intake
		operatorController
				.a()
				.onTrue(
						new CMD_Superstructure(
								superstructure, SuperstructureState.CORAL_STATION)); // A - Intake coral

		// Superstructure Controls
		operatorController.b().onTrue(new CMD_Superstructure(superstructure, SuperstructureState.IDLE));

		// Climbing

		//	operatorController
		//		.leftStick()
		//		.onTrue(new CMD_Elevator(elevator, led, SuperstructureState.CLIMB));
	}

	public Command getAutonomousCommand() {
		return swerve.getAutonomousCommand("RIGHT_4L4");
	}
}
