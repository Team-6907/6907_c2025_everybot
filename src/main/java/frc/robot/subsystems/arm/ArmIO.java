package frc.robot.subsystems.arm;

import frc.robot.subsystems.arm.Arm.Armposition;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double positionDegree = 0.0;
    public double velocityDegreePerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double goal_setpoint = 0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void ArmtoPose(Armposition p) {}

  public default void resetPosion() {}
}
