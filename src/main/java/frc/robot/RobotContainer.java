// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.coral.CMD_ElevatorCoral;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.generic.CMD_Eject;
import frc.robot.commands.generic.CMD_IntakeRace;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.climb.IO_ClimbReal;
import frc.robot.subsystems.climb.SUB_Climb;
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
import frc.robot.subsystems.vision.IO_VisionCamera;
import frc.robot.subsystems.vision.SUB_Vision;

@SuppressWarnings("unused")
public class RobotContainer {

	// Current hard-coded auto
	public static final String AUTO_NAME = "Left_3P";

	// Controller Configuration
	private CommandXboxController driverController;
	private CommandXboxController operatorController;

	// Sendable Choosers
	///	private SendableChooser<String> autoChooser;
	private SendableChooser<Boolean> isRedChooser;

	// Subsystems
	private Drive drive;
	private SUB_Intake intake;
	private SUB_Vision vision;
	private SUB_Elevator elevator;
	private SUB_Superstructure superstructure;
	private SUB_Led led = new SUB_Led(1, 62, AUTO_NAME);

	private SUB_Climb climb;

	public static Command AUTO_COMMAND;

	// private AlignmentCamera alignmentCamera;

	// private Music orchestra;

	public RobotContainer() {

		// Initialize Robot Componets
		initializeControllers();
		initializeSubsystems();
		configurePathplannerCommands();
		configureButtonBindings();

		// Add alliance selector
		isRedChooser.addOption("Red", true);
		isRedChooser.addOption("Blue", false);

		isRedChooser.setDefaultOption("Red", true);
		SmartDashboard.putData("Alliance", isRedChooser);

		// Create auto stuff
		AUTO_COMMAND = AutoBuilder.buildAuto(AUTO_NAME);

		CameraServer.startAutomaticCapture();

		// CameraServer.startAutomaticCapture();
		// alignmentCamera = new AlignmentCamera(0, "Driver CAM");
	}

	private void initializeControllers() {
		driverController = new CommandXboxController(0);
		operatorController = new CommandXboxController(1);
	}

	private void initializeSubsystems() {

		intake = new SUB_Intake(new IO_IntakeReal());
		elevator = new SUB_Elevator(new IO_ElevatorReal());
		climb = new SUB_Climb(new IO_ClimbReal());

		CanandEventLoop.getInstance();

		switch (RobotConstants.ROBOT_MODE) {
			case REAL:
				// Real robot, instantiate hardware IO implementations
				drive =
						new Drive(
								new IO_GyroReal(),
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
								new IO_ModuleBase() {},
								new IO_ModuleBase() {},
								new IO_ModuleBase() {},
								new IO_ModuleBase() {});
				break;
		}

		vision =
				new SUB_Vision(
						drive::addVisionMeasurement,
						new IO_VisionCamera(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
						new IO_VisionCamera(VisionConstants.camera1Name, VisionConstants.robotToCamera1));

		superstructure = new SUB_Superstructure(drive, intake, elevator, led, operatorController);

		// Setup Sendable Choosers
		isRedChooser = new SendableChooser<Boolean>();

		// Set up SysId routines
		/*
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
				*/
	}

	private void configurePathplannerCommands() {

		NamedCommands.registerCommand(
				"Intake_Coral", new CMD_Superstructure(superstructure, SuperstructureState.CORAL_STATION));

		NamedCommands.registerCommand(
				"Intake_Race",
				new CMD_IntakeRace(intake)
						.andThen(new CMD_Superstructure(superstructure, SuperstructureState.IDLE)));

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
				"L4", new CMD_Superstructure(superstructure, SuperstructureState.L4_SCORING_AUTO));

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

		// Coral up
		operatorController.rightBumper().onTrue(new CMD_ElevatorCoral(superstructure, true));

		// Coral Down
		operatorController.leftBumper().onTrue(new CMD_ElevatorCoral(superstructure, false));

		// L4 Quick
		operatorController
				.rightTrigger()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.L4_SCORING_TELE));

		// L3 Quick
		operatorController
				.leftTrigger()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.L3_SCORING));

		// Algae Dynamic
		operatorController.y().onTrue(superstructure.dynamicAlage());

		operatorController
				.povUp()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.ALGAE_BARGE));

		operatorController
				.povDown()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.ALGAE_GROUND));

		// Eject
		operatorController.x().onTrue(new CMD_Eject(superstructure));

		// Intake
		operatorController
				.a()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.CORAL_STATION));

		// Idle
		operatorController.b().onTrue(new CMD_Superstructure(superstructure, SuperstructureState.IDLE));

		// First Auto Align
		driverController
				.button(1)
				.onChange(
						DriveCommands.driveAlign(
								drive,
								() -> SUB_Superstructure.globalFirstPose,
								// () -> isRedChooser.getSelected(),
								driverController,
								elevator.getHeight()));

		// Second Auto Align
		driverController
				.button(4)
				.onChange(
						DriveCommands.driveAlign(
								drive,
								() -> SUB_Superstructure.globalSecondPose,
								// () -> isRedChooser.getSelected(),
								driverController,
								elevator.getHeight()));

		// Climb Automatic
		operatorController
				.povRight()
				.onTrue(
						climb.climbSequence(
								() -> operatorController.povRight().getAsBoolean(), 1.0, led, superstructure));
		// Climb zero
		operatorController.povLeft().onTrue(climb.goToPosition(0, 1));
	}

	public Command getAutonomousCommand() {

		if (AUTO_COMMAND != null) {
			return AUTO_COMMAND;
		} else {
			return new PrintCommand("Auto Command is NULL!");
		}
	}
}
