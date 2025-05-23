// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public MutAngle leftPositionRad = Radians.mutable(0);
    public MutDistance leftPositionMeter = Meters.mutable(0);
    public MutAngularVelocity leftVelocityRadPerSec = RadiansPerSecond.mutable(0);
    public MutLinearVelocity leftVelocityMetPerSec = MetersPerSecond.mutable(0);
    public MutLinearVelocity leftSetpointVelocityMetPerSec = MetersPerSecond.mutable(0);
    public MutVoltage leftAppliedVolts = Volts.mutable(0);

    public MutAngle rightPositionRad = Radians.mutable(0);
    public MutDistance rightPositionMeter = Meters.mutable(0);
    public MutAngularVelocity rightVelocityRadPerSec = RadiansPerSecond.mutable(0);
    public MutLinearVelocity rightVelocityMetPerSec = MetersPerSecond.mutable(0);
    public MutLinearVelocity rightSetpointVelocityMetPerSec = MetersPerSecond.mutable(0);
    public MutVoltage rightAppliedVolts = Volts.mutable(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(LinearVelocity leftRadPerSec, LinearVelocity rightRadPerSec) {}

  public default void resetPosition() {}

  public default void stop() {}

  public default void setPID(double kP, double Ki, double Kd) {}
}
