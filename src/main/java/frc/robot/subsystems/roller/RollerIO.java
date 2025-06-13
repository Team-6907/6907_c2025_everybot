// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public MutAngle position = Rotations.mutable(0.0);
    public MutAngularVelocity velocity = RotationsPerSecond.mutable(0.0);
    public MutVoltage appliedVolts = Volts.mutable(0.0);
    public MutCurrent currentAmps = Amps.mutable(0.0);
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(RollerIOInputs inputs) {}

  public default void runVelocity(AngularVelocity angularVelocity) {}

  public default void runVolts(Voltage volts) {}

  public default void runSetpoint(AngularVelocity angularVelocity) {}
}
