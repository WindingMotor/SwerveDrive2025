package frc.robot.subsystems.swerve;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveInputsAutoLogged extends IO_SwerveBase.SwerveInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Pose", pose);
    table.put("Heading", heading);
    table.put("Velocity", velocity);
  }

  @Override
  public void fromLog(LogTable table) {
    pose = table.get("Pose", pose);
    heading = table.get("Heading", heading);
    velocity = table.get("Velocity", velocity);
  }

  public SwerveInputsAutoLogged clone() {
    SwerveInputsAutoLogged copy = new SwerveInputsAutoLogged();
    copy.pose = this.pose;
    copy.heading = this.heading;
    copy.velocity = this.velocity;
    return copy;
  }
}
