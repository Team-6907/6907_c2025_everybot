// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private static LoggedTunableNumber intakePosition =
      new LoggedTunableNumber("Arm/intakePosition", 0);
  private static LoggedTunableNumber outtake_startPosition =
      new LoggedTunableNumber("Arm/outtake_startPosition", 0);
  private static LoggedTunableNumber outtake_stopPosition =
      new LoggedTunableNumber("Arm/outtake_stopPosition", 0);
  private static LoggedTunableNumber stow_nonePosition =
      new LoggedTunableNumber("Arm/stow_nonePosition", 0);
  private static LoggedTunableNumber stow_inPosition =
      new LoggedTunableNumber("Arm/stow_inPosition", 0);

  private Angle setPoint = Rotations.of(0);
  private Angle currentGoal = Rotations.of(0);

  public enum ArmState {
    INTAKE(Rotations.of(intakePosition.get())),
    OUTTAKE_START(Rotations.of(outtake_startPosition.get())),
    OUTTAKE_STOP(Rotations.of(outtake_stopPosition.get())),
    STOW_NONE(Rotations.of(stow_nonePosition.get())),
    STOW_IN(Rotations.of(stow_inPosition.get()));

    private Angle setpoint;

    ArmState(Angle setpoint) {
      this.setpoint = setpoint;
    }

    public Angle getSetpoint() {
      return setpoint;
    }

    public void updateSetpoint(double setpoint) {
      this.setpoint = Rotations.of(setpoint);
    }
  }

  public Arm(ArmIO io) {
    this.io = io;
    io.resetPosition();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (tuningMode
        && (intakePosition.hasChanged(hashCode())
            || outtake_startPosition.hasChanged(hashCode())
            || outtake_stopPosition.hasChanged(hashCode())
            || stow_nonePosition.hasChanged(hashCode())
            || stow_inPosition.hasChanged(hashCode()))) {
      ArmState.INTAKE.updateSetpoint(intakePosition.get());
      ArmState.OUTTAKE_START.updateSetpoint(outtake_startPosition.get());
      ArmState.OUTTAKE_STOP.updateSetpoint(outtake_stopPosition.get());
      ArmState.STOW_NONE.updateSetpoint(stow_nonePosition.get());
      ArmState.STOW_IN.updateSetpoint(stow_inPosition.get());
    }

    if (currentGoal != setPoint) {
      currentGoal = setPoint;
      io.runSetpoint(currentGoal);
      Logger.recordOutput("Arm/CurrentGoal", currentGoal.in(Rotations));
    }

    Logger.processInputs("Arm", inputs);
  }

  public void setSetpoint(ArmState targetState) {
    this.setPoint = targetState.getSetpoint();
    Logger.recordOutput("Arm/Setpoint", setPoint.in(Rotations));
  }

  public Command setSetpointCommand(ArmState targetState) {
    return Commands.runOnce(() -> setSetpoint(targetState));
  }

  public Command resetPosition() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              io.stop();
              io.resetPosition();
            }),
        Commands.runOnce(
            () -> {
              setPoint = Rotations.of(0);
              currentGoal = Rotations.of(0);
            }));
  }
}
