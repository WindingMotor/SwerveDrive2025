// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SUB_Climb extends SubsystemBase {

	private final IO_ClimbBase io;
	private final IO_ClimbBase.ClimbInputs inputs = new IO_ClimbBase.ClimbInputs();

	public SUB_Climb(IO_ClimbBase io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);

		Logger.processInputs("Climb", inputs);
	}

	public void setMotorSpeed(double speed) {
		io.setMotorSpeed(speed);
	}

	public double getMotorPosition() {
		return inputs.motorPosition;
	}

	/**
	 * Creates a command that moves the climb motor to a specific position. Automatically determines
	 * the direction based on current position.
	 *
	 * @param targetPosition The target position to move to
	 * @param speed The absolute speed value to use (0.0 to 1.0)
	 * @return A command that completes when the target position is reached
	 */
	public Command goToPosition(double targetPosition, double speed) {
		return this.runEnd(
						() -> {
							// Get current position
							double currentPosition = getMotorPosition();
							// Calculate direction based on current vs target position
							double direction = (targetPosition > currentPosition) ? 1.0 : -1.0;
							// Apply direction to speed
							setMotorSpeed(speed * direction);
						},
						() -> {
							// Stop motor when command ends
							setMotorSpeed(0);
						})
				.until(
						() -> {
							// Get current position for comparison
							double currentPosition = getMotorPosition();
							// Check if we've reached or passed the target position
							if (targetPosition > inputs.motorPosition) {
								// Moving up, finish when we reach or exceed target
								return currentPosition >= targetPosition;
							} else {
								// Moving down, finish when we reach or go below target
								return currentPosition <= targetPosition;
							}
						});
	}

	/**
	 * Creates a command that sets the climb motor speed.
	 *
	 * @param speed The speed to set the motor to
	 * @return A command that sets the motor speed
	 */
	public Command setSpeed(double speed) {
		return Commands.runOnce(
				() -> {
					setMotorSpeed(speed);
				});
	}
}
