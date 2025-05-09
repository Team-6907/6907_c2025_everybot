// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {    
    public MutAngle position = Degrees.mutable(0);
    public MutAngularVelocity velocity = DegreesPerSecond.mutable(0);

    public MutVoltage appliedVolts = Volts.mutable(0);
    public MutCurrent currentAmps = Amps.mutable(0);

    public MutAngle setpointPosition = Degrees.mutable(0);
    public MutAngularVelocity setpointVelocity = DegreesPerSecond.mutable(0);
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void runSetpoint(Angle position) {}

  public default void runVolts(Voltage voltage) {}

  public default void resetPosition(Angle position) {}

  public default void setPID(double p, double i, double d) {}

  public default void setFF(double kS, double kG, double kV, double kA) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void stop() {}
}
