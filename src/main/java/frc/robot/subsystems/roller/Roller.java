// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class Roller extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public Roller(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.runVolts(Volts.of((percent * 12.0))), () -> io.runVolts(Volts.of((0.0))));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.runVolts(Volts.of(((forward.getAsDouble() - reverse.getAsDouble()) * 12.0))),
        () -> io.runVolts(Volts.of((0.0))));
  }
}
