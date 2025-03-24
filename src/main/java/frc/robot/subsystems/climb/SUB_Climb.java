// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.subsystems.led.SUB_Led;
import frc.robot.subsystems.superstructure.SUB_Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.math.ExpDecayFF;
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
		final ExpDecayFF controller = new ExpDecayFF(14, 5, .05);

		return this.runEnd(
						() -> {
							// Get current position
							double currentPosition = getMotorPosition();

							setMotorSpeed(speed * controller.calculate(currentPosition, targetPosition));
						},
						() -> {
							// Stop motor when command ends
							setMotorSpeed(0);
						})
				.until(
						() -> {
							// Get current position for comparison
							double currentPosition = getMotorPosition();

							// Return true when within the deadband to stop the command
							return controller.atTarget(currentPosition, targetPosition);
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

	/**
	 * Creates a command sequence for the climbing operation with the following steps: 1. Reset
	 * encoder (start at 0rot) 2. Go to -12.0rot to pop out 3. Go to -3.11rot to start hook on 4. Wait
	 * for confirmation button press 5. Pull down to -11.8rot
	 *
	 * @param confirmDownButton A boolean supplier that returns true when the confirm button is
	 *     pressed
	 * @param motorSpeed The speed to use for motor movements (0.0 to 1.0)
	 * @return A sequential command that executes the full climbing sequence
	 */
	public Command climbSequence(
			java.util.function.BooleanSupplier confirmDownButton,
			double motorSpeed,
			SUB_Led led,
			SUB_Superstructure superstructure) {
		return Commands.sequence(

				// Go Out

				new CMD_Superstructure(superstructure, SuperstructureState.CLIMB),
				goToPosition(-3.53, motorSpeed),

				// WAIT
				Commands.waitUntil(confirmDownButton),

				// Go Down
				goToPosition(5.8, motorSpeed));

		// Climb mode LED ready
		//	led.setClimbState(Pair.of(true, led.PUB_climbReady)),

		// Step 3: Go to setpoint to start hook on
		// goToPosition(-6.0, motorSpeed),

		// Step 4: Wait for confirmation button press before continuing
		// Commands.waitUntil(confirmDownButton),

		// Climb mode LED ready
		// led.setClimbState(Pair.of(true, led.PUB_climbGo)),

		// Step 5: Pull down to -11.8rot
		// goToPosition(-11.85, motorSpeed));
	}
}
