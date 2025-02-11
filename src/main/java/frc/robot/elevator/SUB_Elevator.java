// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.superstructure.SuperstructureState;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.SignalLogger;

public class SUB_Elevator extends SubsystemBase {
    private final IO_ElevatorBase io;
    private final IO_ElevatorBase.ElevatorInputs inputs = new IO_ElevatorBase.ElevatorInputs();
    private SuperstructureState.State localState = SuperstructureState.IDLE;
    private final SysIdRoutine sysIdRoutine;

    public SUB_Elevator(IO_ElevatorBase io) {
        this.io = io;

        // Configure SysId routine
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Default ramp rate (1V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4V
                null, // Default timeout (10s)
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setVoltage(volts.in(Volts)),
                null,
                this
            )
        );
    }

    @Override
    public void periodic() {
        io.setPositionM(localState.getHeightM());
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    // SysId test commands
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
}

