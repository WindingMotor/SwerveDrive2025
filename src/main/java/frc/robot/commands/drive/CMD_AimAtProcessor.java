// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SUB_Swerve;

public class CMD_AimAtProcessor extends Command {
	private final SUB_Swerve swerveSubsystem;
	private final double tolerance;

	/**
	 * Create a command to aim the robot at the speaker.
	 *
	 * @param swerveSubsystem The swerve drive subsystem
	 * @param tolerance Tolerance in degrees for aiming
	 */
	public CMD_AimAtProcessor(SUB_Swerve swerveSubsystem, double tolerance) {
		this.swerveSubsystem = swerveSubsystem;
		this.tolerance = tolerance;

		// Require the swerve subsystem
		addRequirements(swerveSubsystem);
	}

	@Override
	public void execute() {
		ChassisSpeeds speeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(
						0,
						0,
						swerveSubsystem
								.getSwerveController()
								.headingCalculate(
										swerveSubsystem.getHeading().getRadians(),
										swerveSubsystem.getProcessorYaw().getRadians()),
						swerveSubsystem.getHeading());

		swerveSubsystem.drive(speeds);
	}

	@Override
	public boolean isFinished() {
		return Math.abs(
						swerveSubsystem.getProcessorYaw().minus(swerveSubsystem.getHeading()).getDegrees())
				< tolerance;
	}

	@Override
	public void end(boolean interrupted) {
		// Stop the robot when the command ends
		swerveSubsystem.drive(new ChassisSpeeds());
	}
}
