// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private static LoggedTunableNumber intakeVelocity =
      new LoggedTunableNumber("Roller/intakeVelocity", 0);
  private static LoggedTunableNumber outtakeVelocity =
      new LoggedTunableNumber("Roller/outtakeVelocity", 0);

  private AngularVelocity setPoint = RotationsPerSecond.of(0);
  private AngularVelocity currentGoal = RotationsPerSecond.of(0);

  public enum RollerState {
    STOW(RotationsPerSecond.of(0)),
    INTAKE(RotationsPerSecond.of(intakeVelocity.get())),
    OUTTAKE(RotationsPerSecond.of(outtakeVelocity.get()));

    private AngularVelocity setpoint;

    RollerState(AngularVelocity setpoint) {
      this.setpoint = setpoint;
    }

    public AngularVelocity getSetpoint() {
      return setpoint;
    }

    public void updateSetpoint(double setpoint) {
      this.setpoint = RotationsPerSecond.of(setpoint);
    }
  }

  public Roller(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (tuningMode
        && (intakeVelocity.hasChanged(hashCode()) || outtakeVelocity.hasChanged(hashCode()))) {
      RollerState.INTAKE.updateSetpoint(intakeVelocity.get());
      RollerState.OUTTAKE.updateSetpoint(outtakeVelocity.get());
    }

    if (currentGoal != setPoint) {
      currentGoal = setPoint;
      io.runVolts(Volts.of(currentGoal.in(RotationsPerSecond)));
      Logger.recordOutput("Arm/CurrentGoal", currentGoal.in(RotationsPerSecond));
    }

    Logger.processInputs("Roller", inputs);
  }

  public void setSetpoint(RollerState targetState) {
    this.setPoint = targetState.getSetpoint();
    Logger.recordOutput("Roller/Setpoint", setPoint.in(RotationsPerSecond));
  }

  public Command setSetpointCommand(RollerState targetState) {
    return Commands.runOnce(() -> setSetpoint(targetState));
  }

  public Command runPercent(double percent) {
    return runEnd(
        () -> io.runVolts(Volts.of((percent * 12.0))), () -> io.runVolts(Volts.of((0.0))));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.runVolts(Volts.of(((forward.getAsDouble() - reverse.getAsDouble()) * 12.0))),
        () -> io.runVolts(Volts.of((0.0))));
  }
}
