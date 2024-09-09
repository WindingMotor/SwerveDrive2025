// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SUB_Swerve extends SubsystemBase {

	private final IO_SwerveBase io;

	private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

	// Odometry lock, prevents updates while reading data
	private static final Lock odometryLock = new ReentrantLock();

	/**
	 * Constructs a new SUB_Swerve instance.
	 *
	 * @param io The IO_SwerveBase implementation to use
	 * @param driverController The driver's controller (currently unused in this implementation)
	 */
	public SUB_Swerve(IO_SwerveBase io, CommandXboxController driverController) {
		this.io = io;

		configureAutoBuilder();
	}

	/** Configures the AutoBuilder for autonomous path following. */
	private void configureAutoBuilder() {
		AutoBuilder.configureHolonomic(
				this::getPose,
				io::resetOdometry,
				this::getRobotRelativeSpeeds,
				io::setChassisSpeeds,
				new HolonomicPathFollowerConfig(
						inputs.getTranslationPID(),
						inputs.getHeadingPID(),
						4.5,
						inputs.configurationRadius,
						new ReplanningConfig()),
				() ->
						DriverStation.getAlliance().isPresent()
								&& DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
				this);
	}

	@Override
	public void periodic() {
		odometryLock.lock();
		try {
			io.updateInputs(inputs);
			io.updateOdometry();
		} finally {
			odometryLock.unlock();
		}

		Logger.processInputs("Swerve", inputs);
	}

	/**
	 * Creates a command to drive the robot using joystick inputs.
	 *
	 * @param translationX X-axis translation input
	 * @param translationY Y-axis translation input
	 * @param thetaZ Rotation input
	 * @return A command that drives the robot based on the given inputs
	 */
	public Command drive(
			DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier thetaZ) {
		return run(
				() -> {
					var alliance = DriverStation.getAlliance();
					double xSpeed = translationX.getAsDouble() * inputs.maximumVelocity;
					double ySpeed = translationY.getAsDouble() * inputs.maximumVelocity;
					double thetaSpeed = -thetaZ.getAsDouble() * inputs.maximumAngularVelocity;

					if (alliance.isPresent() && alliance.get() == Alliance.Red) {
						xSpeed = -xSpeed;
						ySpeed = -ySpeed;
					}

					io.drive(new Translation2d(xSpeed, ySpeed), thetaSpeed, true, true);
				});
	}

	/**
	 * Creates a command to drive the robot to a specified pose.
	 *
	 * @param pose The target pose to drive to
	 * @return A command that drives the robot to the specified pose
	 */
	public Command driveToPose(Pose2d pose) {
		return AutoBuilder.pathfindToPose(
				pose, new PathConstraints(4.0, 3.0, Math.toRadians(540), Math.toRadians(720)));
	}

	/**
	 * Helper for pathplanner that gets the current pose of the robot.
	 *
	 * @return The current pose of the robot
	 */
	private Pose2d getPose() {
		return inputs.pose;
	}

	/**
	 * Helper for pathplanner that gets the current robot-relative chassis speeds.
	 *
	 * @return The robot-relative chassis speeds
	 */
	private ChassisSpeeds getRobotRelativeSpeeds() {
		return inputs.velocity;
	}
}
