// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.algae.CMD_ElevatorAlgae;
import frc.robot.commands.coral.CMD_ElevatorCoral;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveCommands.ZonePose;
import frc.robot.commands.generic.CMD_Eject;
import frc.robot.commands.generic.CMD_IntakeRace;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.constants.InputConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.IO_GyroBase;
import frc.robot.subsystems.drive.IO_GyroReal;
import frc.robot.subsystems.drive.IO_ModuleBase;
import frc.robot.subsystems.drive.IO_ModuleReal;
import frc.robot.subsystems.drive.IO_ModuleSim;
import frc.robot.subsystems.elevator.IO_ElevatorReal;
import frc.robot.subsystems.elevator.SUB_Elevator;
import frc.robot.subsystems.intake.IO_IntakeReal;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.led.SUB_Led;
import frc.robot.subsystems.superstructure.SUB_Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.vision.IO_VisionReal;
import frc.robot.subsystems.vision.SUB_Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
	// Controller Configuration
	private CommandXboxController driverController;
	private CommandXboxController operatorController;
	private InputConstants globalInputMap;

	private Drive drive;
	private LoggedDashboardChooser<Command> autoChooser;

	// Subsystems
	private SUB_Intake intake;
	private SUB_Vision vision;
	private SUB_Elevator elevator;
	private SUB_Superstructure superstructure;
	private final SUB_Led led = new SUB_Led(1, 62);

	// private Music orchestra;

	public RobotContainer() {
		// Initialize Controllers
		initializeControllers();

		// Initialize Subsystems
		initializeSubsystems();

		configurePathplannerCommands();

		// Configure Robot Functionality
		configureWebserverCommands();
		configureButtonBindings();
	}

	private void initializeControllers() {
		driverController = new CommandXboxController(0);
		operatorController = new CommandXboxController(1);
		globalInputMap = InputConstants.TX16S_MAIN;
	}

	private void initializeSubsystems() {
		vision = new SUB_Vision(new IO_VisionReal());
		intake = new SUB_Intake(new IO_IntakeReal());
		elevator = new SUB_Elevator(new IO_ElevatorReal());

		// CommandRegistrar.registerCommands(swerve, superstructure);
		CanandEventLoop.getInstance();

		switch (RobotConstants.ROBOT_MODE) {
			case REAL:
				// Real robot, instantiate hardware IO implementations
				drive =
						new Drive(
								new IO_GyroReal(),
								vision,
								new IO_ModuleReal(TunerConstants.FrontLeft),
								new IO_ModuleReal(TunerConstants.FrontRight),
								new IO_ModuleReal(TunerConstants.BackLeft),
								new IO_ModuleReal(TunerConstants.BackRight));
				break;

			case SIM:
				// Sim robot, instantiate physics sim IO implementations
				drive =
						new Drive(
								new IO_GyroBase() {},
								vision,
								new IO_ModuleSim(TunerConstants.FrontLeft),
								new IO_ModuleSim(TunerConstants.FrontRight),
								new IO_ModuleSim(TunerConstants.BackLeft),
								new IO_ModuleSim(TunerConstants.BackRight));
				break;

			default:
				// Replayed robot, disable IO implementations
				drive =
						new Drive(
								new IO_GyroBase() {},
								vision,
								new IO_ModuleBase() {},
								new IO_ModuleBase() {},
								new IO_ModuleBase() {},
								new IO_ModuleBase() {});
				break;
		}

		superstructure = new SUB_Superstructure(drive, intake, elevator, led);

		// Set up auto routines
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		// Set up SysId routines
		autoChooser.addOption(
				"Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
		autoChooser.addOption(
				"Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
		autoChooser.addOption(
				"Drive SysId (Quasistatic Forward)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"Drive SysId (Quasistatic Reverse)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption(
				"Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
	}

	private void configureWebserverCommands() {}

	private void configurePathplannerCommands() {

		NamedCommands.registerCommand(
				"Intake_Coral", new CMD_Superstructure(superstructure, SuperstructureState.CORAL_STATION));

		NamedCommands.registerCommand(
				"Intake_Race",
				new CMD_IntakeRace(intake)
						.andThen(new CMD_Superstructure(superstructure, SuperstructureState.IDLE)));

		// BOTTOM RIGHT
		NamedCommands.registerCommand(
				"ALN_BOTTOM_RIGHT_TOP", DriveCommands.driveToZone(drive, ZonePose.REEF_BOTTOM_RIGHT_TOP));

		NamedCommands.registerCommand(
				"ALN_BOTTOM_RIGHT_BOTTOM",
				DriveCommands.driveToZone(drive, ZonePose.REEF_BOTTOM_RIGHT_BOTTOM));

		// TOP RIGHT
		NamedCommands.registerCommand(
				"ALN_REEF_TOP_RIGHT_TOP", DriveCommands.driveToZone(drive, ZonePose.REEF_TOP_RIGHT_TOP));

		NamedCommands.registerCommand(
				"ALN_TOP_RIGHT_BOTTOM", DriveCommands.driveToZone(drive, ZonePose.REEF_TOP_RIGHT_BOTTOM));

		// BOTTOM
		NamedCommands.registerCommand(
				"ALN_REEF_BOTTOM_LEFT", DriveCommands.driveToZone(drive, ZonePose.REEF_BOTTOM_LEFT));

		NamedCommands.registerCommand(
				"ALN_REEF_BOTTOM_RIGHT", DriveCommands.driveToZone(drive, ZonePose.REEF_BOTTOM_RIGHT));

		// SOURCE
		NamedCommands.registerCommand(
				"ALN_SOURCE_RIGHT", DriveCommands.driveToZone(drive, ZonePose.SOURCE_RIGHT));

		NamedCommands.registerCommand(
				"ALN_SOURCE_LEFT", DriveCommands.driveToZone(drive, ZonePose.SOURCE_LEFT));

		// //

		NamedCommands.registerCommand(
				"L1", new CMD_Superstructure(superstructure, SuperstructureState.L1_SCORING));

		NamedCommands.registerCommand(
				"L2", new CMD_Superstructure(superstructure, SuperstructureState.L2_SCORING));

		NamedCommands.registerCommand(
				"L2C", new CMD_Superstructure(superstructure, SuperstructureState.L2_CLEAR));

		NamedCommands.registerCommand(
				"L3", new CMD_Superstructure(superstructure, SuperstructureState.L3_SCORING));

		NamedCommands.registerCommand(
				"L3C", new CMD_Superstructure(superstructure, SuperstructureState.L3_CLEAR));

		NamedCommands.registerCommand(
				"L4", new CMD_Superstructure(superstructure, SuperstructureState.L4_SCORING));

		NamedCommands.registerCommand(
				"L4C", new CMD_Superstructure(superstructure, SuperstructureState.L4_CLEAR));

		NamedCommands.registerCommand("Eject", new CMD_Eject(superstructure));

		NamedCommands.registerCommand(
				"Idle", new CMD_Superstructure(superstructure, SuperstructureState.IDLE));
	}

	private void configureButtonBindings() {

		// Drive w/ Assist Rotation
		drive.setDefaultCommand(
				DriveCommands.driveWithAssist(
						drive,
						() -> -driverController.getRawAxis(1),
						() -> driverController.getRawAxis(0),
						() -> -driverController.getRawAxis(3),
						() -> driverController.button(3).getAsBoolean()));

		/*
		drive.setDefaultCommand(
				DriveCommands.driveNormal(
						drive,
						() -> driverController.getRawAxis(1),
						() -> -driverController.getRawAxis(0),
						() -> -driverController.getRawAxis(3)));
		*/

		// Eject
		operatorController.x().onTrue(new CMD_Eject(superstructure));

		// Coral Raise L1-to-L4
		operatorController
				.rightBumper()
				.onTrue(new CMD_ElevatorCoral(superstructure, true)); // DPAD-UP - Coral up

		// L4 Quick
		operatorController
				.rightTrigger()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.L4_SCORING));

		// L3 Quick
		operatorController
				.leftTrigger()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.L3_SCORING));

		// Algae Raise AL2-AL3
		operatorController.leftBumper().onTrue(new CMD_ElevatorAlgae(superstructure, true));

		// Intake
		operatorController
				.a()
				.onTrue(
						new CMD_IntakeRace(intake)
								.andThen(new CMD_Superstructure(superstructure, SuperstructureState.IDLE)));

		// Idle
		operatorController.b().onTrue(new CMD_Superstructure(superstructure, SuperstructureState.IDLE));

		// Climb Raise
		// operatorController.leftStick().onTrue(new CMD_Elevator(elevator, SuperstructureState.CLIMB));

		// Climb Lower

		// operatorController
		//		.rightStick()
		//		.onTrue(new CMD_Elevator(elevator, SuperstructureState.CLIMB_BTM));

		// Auto align test
		// operatorController.rightStick().onTrue(DriveCommands.driveToZone(drive,
		// ZonePose.SOURCE_LEFT));

		operatorController
				.leftStick()
				.onTrue(DriveCommands.driveToZone(drive, ZonePose.REEF_BOTTOM_LEFT));

		operatorController
				.rightStick()
				.onTrue(DriveCommands.driveToZone(drive, ZonePose.REEF_BOTTOM_RIGHT_TOP));

		operatorController
				.povDown()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.ALGAE_GROUND));
	}

	public Command getAutonomousCommand() {
		// return swerve.getAutonomousCommand("T1");
		return AutoBuilder.buildAuto("T1A");
	}
}
