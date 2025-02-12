// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import swervelib.SwerveController;
import swervelib.parser.SwerveDriveConfiguration;

public interface IO_SwerveBase {
	@AutoLog
	public static class SwerveInputs implements LoggableInputs {
		// Robot pose and orientation
		public Pose2d robotPose = new Pose2d();
		public double gyroYawDegrees = 0;
		public double gyroYawRateDegreesPerSec = 0;
		public Rotation2d gyroYaw = new Rotation2d();
		public Rotation2d gyroPitch = new Rotation2d();

		// Module states
		public SwerveModuleState[] moduleStates = new SwerveModuleState[4];
		public SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

		// Chassis speeds
		public ChassisSpeeds fieldSpeeds = new ChassisSpeeds();
		public ChassisSpeeds robotSpeeds = new ChassisSpeeds();

		// Driving parameters
		public boolean isFieldRelative = true;
		public boolean isOpenLoop = false;

		@Override
		public void toLog(LogTable table) {
			table.put("RobotPose", robotPose);
			table.put("gyroYawDegrees", gyroYawDegrees);
			table.put("gyroYawRateDegreesPerSec", gyroYawRateDegreesPerSec);
			table.put("GyroYaw", gyroYaw);
			table.put("GyroPitch", gyroPitch);
			table.put("ModuleStates", moduleStates);
			table.put("ModulePositions", modulePositions);
			table.put("FieldSpeeds", fieldSpeeds);
			table.put("RobotSpeeds", robotSpeeds);
			table.put("IsFieldRelative", isFieldRelative);
			table.put("IsOpenLoop", isOpenLoop);
		}

		@Override
		public void fromLog(LogTable table) {
			robotPose = table.get("RobotPose", robotPose);
			gyroYawDegrees = table.get("gyroYawDegrees", gyroYawDegrees);
			gyroYawRateDegreesPerSec = table.get("gyroYawRateDegreesPerSec", gyroYawRateDegreesPerSec);
			gyroYaw = table.get("GyroYaw", gyroYaw);
			gyroPitch = table.get("GyroPitch", gyroPitch);
			moduleStates = table.get("ModuleStates", moduleStates);
			modulePositions = table.get("ModulePositions", modulePositions);
			fieldSpeeds = table.get("FieldSpeeds", fieldSpeeds);
			robotSpeeds = table.get("RobotSpeeds", robotSpeeds);
			isFieldRelative = table.get("IsFieldRelative", isFieldRelative);
			isOpenLoop = table.get("IsOpenLoop", isOpenLoop);
		}
	}

	/** Updates the set of loggable inputs. */
	public void updateInputs(SwerveInputs inputs);

	/** Drive the swerve with translation and rotation. */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative);

	/** Drive with robot-oriented chassis speeds. */
	public void drive(ChassisSpeeds velocity);

	/** Set chassis speeds directly. */
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds);

	/** Reset the odometry to a specific pose. */
	public void resetOdometry(Pose2d pose);

	/** Zero the gyro. */
	public void zeroGyro();

	/** Set motor brake mode. */
	public void setMotorBrake(boolean brake);

	/** Get the current pose. */
	public Pose2d getPose();

	/** Get the current heading. */
	public Rotation2d getHeading();

	/** Get the current pitch. */
	public Rotation2d getPitch();

	/** Get the field-relative velocity. */
	public ChassisSpeeds getFieldVelocity();

	/** Get the robot-relative velocity. */
	public ChassisSpeeds getRobotVelocity();

	/** Get the swerve drive kinematics. */
	public SwerveDriveKinematics getKinematics();

	/** Get the swerve controller. */
	public SwerveController getSwerveController();

	/** Get the swerve drive configuration. */
	public SwerveDriveConfiguration getSwerveDriveConfiguration();

	/** Lock the swerve drive. */
	public void lock();

	/** Post a trajectory to the field. */
	public void postTrajectory(Trajectory trajectory);

	/** Setup PathPlanner. */
	public void setupPathPlanner(SUB_Swerve swerveSubsystem);

	public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);

	public Command setAllAngle(double angle);
}
