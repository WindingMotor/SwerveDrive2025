// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.SuperstructureState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/*
Conversion factor explained:

METERS_PER_MOTOR_ROTATION represents the linear travel of the elevator per motor rotation.

It is derived using the formula:
	(1 / gearbox ratio) * (PI * pitch circle diameter of the sprocket) * stage multiplier
- The gearbox ratio determines how much the motor turns relative to the output.
- PI * pitch circle diameter gives the circumference of the sprocket, corresponding to
	the linear movement for one rotation.
- The stage multiplier accounts for additional travel as the stage extends (e.g., stage 2
	doubles the first stage, stage 3 triples, etc.).
	For this calculation:
	(1 / 16.25) * (PI * 0.0444754) * 3
	= 0.0257951242902 meters per motor rotation.


*/

public class SUB_Elevator extends SubsystemBase {
	private final IO_ElevatorBase io;
	private final IO_ElevatorBase.ElevatorInputs inputs = new IO_ElevatorBase.ElevatorInputs();
	private SuperstructureState.State localState = SuperstructureState.IDLE;
	private final SysIdRoutine sysIdRoutine;

	public SUB_Elevator(IO_ElevatorBase io) {
		this.io = io;

		SignalLogger.setPath("");

		// Configure SysId routine
		sysIdRoutine =
				new SysIdRoutine(
						new SysIdRoutine.Config(
								null, // Default ramp rate (1V/s)
								Volts.of(4), // Reduce dynamic step voltage to 4V
								null, // Default timeout (10s)
								(state) -> SignalLogger.writeString("state", state.toString())),
						new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));
	}

	@Override
	public void periodic() {
		io.setPositionM(localState.getHeightM());
		io.updateInputs(inputs);
		Logger.processInputs("Elevator", inputs);
	}

	public void setVoltage(double volts) {
		io.setVoltage(volts);
	}

	public Command createSysIdCommand(Command sysIdCommand) {
		return Commands.sequence(sysIdCommand);
	}

	// Simplify SysId commands to just return the routine
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.dynamic(direction);
	}

	public void updateLocalState(SuperstructureState.State newLocalState) {
		localState = newLocalState;
	}

	public SuperstructureState.State getCurrentLocalState() {
		return localState;
	}

	public DoubleSupplier getHeight() {
		return () -> inputs.heightM; // Return the current height in meters
	}
}
