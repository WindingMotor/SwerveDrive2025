// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.generic;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.elevator.SUB_Elevator;
import java.util.concurrent.atomic.AtomicReference;

public class CMD_ElevatorSysID extends Command {
	private final SUB_Elevator elevator;
	private final CommandXboxController operatorController;

	private enum SysIdState {
		WAITING,
		DYN_FORWARD,
		DYN_REVERSE,
		QUASI_FORWARD,
		QUASI_REVERSE,
		COMPLETE
	}

	private final AtomicReference<SysIdState> currentState =
			new AtomicReference<>(SysIdState.WAITING);

	public CMD_ElevatorSysID(SUB_Elevator elevator, CommandXboxController operatorController) {
		this.elevator = elevator;
		this.operatorController = operatorController;
		addRequirements(elevator);
		configureButtons();
	}

	private void configureButtons() {
		operatorController
				.a()
				.onTrue(
						Commands.runOnce(
								() -> {
									SignalLogger.start();
									currentState.set(SysIdState.DYN_FORWARD);
									DriverStation.reportError(
											"SysId Started - Ready for Dynamic Forward Test", false);
								}));

		operatorController
				.x()
				.onTrue(
						Commands.runOnce(
								() -> {
									elevator.setVoltage(0);

									switch (currentState.get()) {
										case DYN_FORWARD:
											elevator.sysIdDynamic(SysIdRoutine.Direction.kForward).schedule();
											currentState.set(SysIdState.DYN_REVERSE);
											DriverStation.reportError(
													"Dynamic Forward Complete - Ready for Dynamic Reverse", false);
											break;

										case DYN_REVERSE:
											elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse).schedule();
											currentState.set(SysIdState.QUASI_FORWARD);
											DriverStation.reportError(
													"Dynamic Reverse Complete - Ready for Quasistatic Forward", false);
											break;

										case QUASI_FORWARD:
											elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward).schedule();
											currentState.set(SysIdState.QUASI_REVERSE);
											DriverStation.reportError(
													"Quasistatic Forward Complete - Ready for Quasistatic Reverse", false);
											break;

										case QUASI_REVERSE:
											elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).schedule();
											currentState.set(SysIdState.COMPLETE);
											DriverStation.reportError(
													"Quasistatic Reverse Complete - All Tests Done!", false);
											break;

										default:
											DriverStation.reportError("No test to run or sequence complete", false);
											break;
									}
								}));

		operatorController
				.b()
				.onTrue(
						Commands.runOnce(
								() -> {
									elevator.setVoltage(0);
									StatusCode stopStatus = SignalLogger.stop();
									currentState.set(SysIdState.WAITING);
									DriverStation.reportError(
											"SysId Stopped - Logger Status: " + stopStatus.toString(), false);
								}));
	}

	@Override
	public void initialize() {
		currentState.set(SysIdState.WAITING);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
