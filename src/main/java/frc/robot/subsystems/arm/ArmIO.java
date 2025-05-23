// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public MutAngle position = Rotations.mutable(0);
    public MutAngularVelocity velocity = RotationsPerSecond.mutable(0);
    public MutVoltage appliedVolts = Volts.mutable(0);
    public MutCurrent currentAmps = Amps.mutable(0);
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void runSetpoint(Angle position) {}

  public default void runVolts(Voltage voltage) {}

  public default void resetPosition() {}

  public default void stop() {}
}
