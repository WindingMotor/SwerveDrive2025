// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.util.ExpDecayFF;
import frc.robot.util.ExpDecayFF.RotationState;
import java.io.File;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class IO_SwerveReal implements IO_SwerveBase {

	private final SwerveDrive swerveDrive;
	private final SwerveInputs inputs = new SwerveInputs();

	private final ExpDecayFF rotationController;
	private double rotationControllerValue = 0.0;

	public IO_SwerveReal(File directory) {
		// Configure the Telemetry before creating the SwerveDrive
		SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.NONE;
		try {
			swerveDrive =
					new SwerveParser(directory)
							.createSwerveDrive(
									RobotConstants.MAX_SPEED,
									new Pose2d(
											new Translation2d(Meter.of(0.01), Meter.of(0.01)),
											Rotation2d.fromDegrees(0)));
		} catch (Exception e) {
			throw new RuntimeException(e);
		}

		// Configure SwerveDrive settings
		// swerveDrive.setHeadingCorrection(false);

		swerveDrive.setCosineCompensator(true);
		swerveDrive.setAngularVelocityCompensation(true, false, -0.03);
		swerveDrive.setModuleEncoderAutoSynchronize(false, 3);

		// swerveDrive.pushOffsetsToEncoders();

		swerveDrive.setVisionMeasurementStdDevs(CameraConstants.VISION_ESTIMATION_STD_DEVS);

		// Stop inteernal odometry thread to increase speeed
		swerveDrive.stopOdometryThread();

		this.rotationController = new ExpDecayFF(6.0, 1, 0.1);

		swerveDrive.setMotorIdleMode(true);
	}

	@Override
	public void updateInputs(SwerveInputs inputs) {
		inputs.robotPose = swerveDrive.getPose();
		inputs.gyroYawRateDegreesPerSec =
				Units.radiansToDegrees(swerveDrive.getRobotVelocity().omegaRadiansPerSecond);
		inputs.gyroYawDegrees = swerveDrive.getYaw().getDegrees();
		inputs.speeds = swerveDrive.getRobotVelocity();

		/*
		 * Before offset applied in AScope
		Logger.recordOutput(
				"ArmSimulationPose",
				new Pose3d(
						0.25, // x offset from robot center
						-0.2, // y offset from robot center
						0.31, // z height
						new Rotation3d(
								Math.toRadians(270), Math.toRadians(180), Math.toRadians(90)) // arm rotation
						));
		 */

		/*
		Logger.recordOutput(
				"ArmSimulationPose",
				new Pose3d(
						0.0, // x offset from robot center
						-0.0, // y offset from robot center
						0.0, // z height
						new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)) // arm rotation
						));
			*/

		Logger.recordOutput("Rotation Controller State", rotationController.getState().toString());

		rotationController.setState(DynamicConstants.GLOBAL_ROTATION_STATE);
		if (rotationController.getState() != RotationState.NONE) {
			rotationControllerValue = rotationController.calculate(-inputs.gyroYawDegrees);
			Logger.recordOutput("Rotation Command", rotationControllerValue);
			Logger.recordOutput("Current Yaw", -inputs.gyroYawDegrees);
			Logger.recordOutput("Current Setpoint", rotationController.getTargetAngle());
		}
	}

	@Override
	public void drive(Translation2d translation, double rotation, boolean fieldRelative) {

		if (DriverStation.isTeleop()) {
			if (rotationController.getState() != RotationState.NONE) {
				swerveDrive.drive(translation, rotationControllerValue, fieldRelative, true);
			} else {
				swerveDrive.drive(translation, rotation, fieldRelative, true);
			}
		} else {
			swerveDrive.drive(translation, -rotation, fieldRelative, true);
		}
	}

	@Override
	public void drive(ChassisSpeeds velocity) {
		swerveDrive.drive(velocity);
		/*
		if (rotationController.getState() != RotationState.NONE) {
			velocity =
					new ChassisSpeeds(
							velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, rotationControllerValue);
		} else {
			velocity = new ChassisSpeeds(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0.0);
		}
		swerveDrive.drive(velocity);
		*/
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		swerveDrive.setChassisSpeeds(chassisSpeeds);
	}

	@Override
	public void resetOdometry(Pose2d initialHolonomicPose) {
		swerveDrive.resetOdometry(new Pose2d(initialHolonomicPose.getTranslation(), new Rotation2d()));
	}

	@Override
	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	@Override
	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	@Override
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	private Pose2d getPosePathPlanner() {
		return new Pose2d(swerveDrive.getPose().getTranslation(), swerveDrive.getYaw());
	}

	@Override
	public Rotation2d getHeading() {
		return swerveDrive.getYaw();
	}

	@Override
	public Rotation2d getPitch() {
		return swerveDrive.getPitch();
	}

	@Override
	public ChassisSpeeds getFieldVelocity() {
		return swerveDrive.getFieldVelocity();
	}

	@Override
	public ChassisSpeeds getRobotVelocity() {
		return swerveDrive.getRobotVelocity();
	}

	@Override
	public SwerveDriveKinematics getKinematics() {
		return swerveDrive.kinematics;
	}

	@Override
	public SwerveController getSwerveController() {
		return swerveDrive.swerveController;
	}

	@Override
	public SwerveDriveConfiguration getSwerveDriveConfiguration() {
		return swerveDrive.swerveDriveConfiguration;
	}

	@Override
	public void lock() {
		swerveDrive.lockPose();
	}

	@Override
	public void postTrajectory(Trajectory trajectory) {
		swerveDrive.postTrajectory(trajectory);
	}

	@Override
	public Command setAllAngle(double angle) {
		return new InstantCommand(
				() -> {
					swerveDrive.getModules()[0].setAngle(angle);
					swerveDrive.getModules()[1].setAngle(angle);
					swerveDrive.getModules()[2].setAngle(angle);
					swerveDrive.getModules()[3].setAngle(angle);
				});
	}

	@Override
	public void setupPathPlanner(SUB_Swerve swerveSubsystem) {

		try {
			// Load PathPlanner config from GUI settings
			RobotConfig config = RobotConfig.fromGUISettings();

			final boolean enableFeedforward = true;

			// Configure AutoBuilder
			AutoBuilder.configure(
					this::getPosePathPlanner, // Robot pose supplier
					this::resetOdometry, // Method to reset odometry
					this::getRobotVelocity, // ChassisSpeeds supplier (MUST BE ROBOT RELATIVE)
					(speedsRobotRelative, moduleFeedForwards) -> {
						if (enableFeedforward) {
							ChassisSpeeds newSpeeds =
									new ChassisSpeeds(
											speedsRobotRelative.vxMetersPerSecond,
											speedsRobotRelative.vyMetersPerSecond,
											speedsRobotRelative.omegaRadiansPerSecond);
							swerveDrive.drive(
									newSpeeds,
									swerveDrive.kinematics.toSwerveModuleStates(newSpeeds),
									moduleFeedForwards.linearForces());
						} else {
							swerveDrive.setChassisSpeeds(speedsRobotRelative);
						}
					},
					new PPHolonomicDriveController(
							new PIDConstants(1.25, 0.0, 0.0), // Translation PID constants
							getHeadingPID()),
					config,
					() -> {
						// Boolean supplier that controls when the path will be mirrored for the red alliance
						var alliance = DriverStation.getAlliance();
						if (alliance.isPresent()) {
							return alliance.get() == DriverStation.Alliance.Red;
						}
						return false;
					},
					swerveSubsystem // Reference to this subsystem to set requirements
					);

		} catch (Exception e) {
			e.printStackTrace();
		}

		// Preload PathPlanner Path finding
		PathfindingCommand.warmupCommand().schedule();
	}

	/**
	 * Add a vision measurement to the swerve drive odometry.
	 *
	 * @param pose The measured pose
	 * @param timestamp The timestamp of the measurement in seconds
	 */
	@Override
	public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
		swerveDrive.addVisionMeasurement(pose, timestamp, stdDevs);
	}

	/** Update odometry for the swerve drive. */
	public void updateOdometry() {
		swerveDrive.updateOdometry();
	}

	private PIDConstants getHeadingPID() {
		return new PIDConstants(
				swerveDrive.swerveController.config.headingPIDF.p, // Rotation PID
				swerveDrive.swerveController.config.headingPIDF.i,
				swerveDrive.swerveController.config.headingPIDF.d);
	}

	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	public ExpDecayFF getRotationController() {
		return rotationController;
	}
}
