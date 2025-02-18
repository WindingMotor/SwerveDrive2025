// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.algae.CMD_ElevatorAlgae;
import frc.robot.commands.coral.CMD_ElevatorCoral;
import frc.robot.commands.generic.CMD_Eject;
import frc.robot.commands.generic.CMD_Elevator;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.constants.InputConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.drive.Drive;
import frc.robot.drive.IO_GyroBase;
import frc.robot.drive.IO_GyroReal;
import frc.robot.drive.IO_ModuleBase;
import frc.robot.drive.IO_ModuleReal;
import frc.robot.drive.IO_ModuleSim;
import frc.robot.elevator.IO_ElevatorReal;
import frc.robot.elevator.SUB_Elevator;
import frc.robot.generated.TunerConstants;
import frc.robot.intake.IO_IntakeReal;
import frc.robot.intake.SUB_Intake;
import frc.robot.superstructure.SUB_Superstructure;
import frc.robot.superstructure.SuperstructureState;
import frc.robot.util.SUB_Led;
import frc.robot.vision.IO_VisionReal;
import frc.robot.vision.SUB_Vision;
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
	private SUB_Led led;

	// private Music orchestra;

	public RobotContainer() {
		// Initialize Controllers
		initializeControllers();

		// Initialize Subsystems
		initializeSubsystems();

		// Configure Robot Functionality
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
		vision = new SUB_Vision(new IO_VisionReal());
		intake = new SUB_Intake(new IO_IntakeReal());
		elevator = new SUB_Elevator(new IO_ElevatorReal());
		led = new SUB_Led();
		superstructure = new SUB_Superstructure(intake, elevator, led);

		// CommandRegistrar.registerCommands(swerve, superstructure);
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

	private void configurePathPlannerCommands() {
		NamedCommands.registerCommand("Intake_Algae", new PrintCommand("Intake Algae"));
	}

	private void configureButtonBindings() {

		drive.setDefaultCommand(
				DriveCommands.joystickDrive(
						drive,
						driverController::getLeftX,
						driverController::getLeftY,
						driverController::getRightX));

		// Switch to X pattern when X button is pressed
		driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

		// Reset gyro to 0° when B button is pressed
		driverController
				.b()
				.onTrue(
						Commands.runOnce(
										() ->
												drive.setPose(
														new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
										drive)
								.ignoringDisable(true));

		// Extake
		operatorController.x().onTrue(new CMD_Eject(superstructure));
		operatorController
				.x()
				.toggleOnFalse(new CMD_Superstructure(superstructure, SuperstructureState.IDLE));

		// Coral Controls
		operatorController
				.rightBumper()
				.onTrue(new CMD_ElevatorCoral(superstructure, true)); // DPAD-UP - Coral up

		operatorController
				.rightTrigger()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.L4_SCORING));
		operatorController
				.leftTrigger()
				.onTrue(new CMD_Superstructure(superstructure, SuperstructureState.L3_SCORING));

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
		operatorController
				.leftStick()
				.onTrue(new CMD_Elevator(elevator, led, SuperstructureState.CLIMB));

		operatorController
				.rightStick()
				.onTrue(new CMD_Elevator(elevator, led, SuperstructureState.CLIMB_BTM));
	}

	public Command getAutonomousCommand() {
		// return swerve.getAutonomousCommand("T1");
		return null;
	}
}
