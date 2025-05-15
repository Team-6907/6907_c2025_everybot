// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private Angle setpoint = Degrees.of(0);
  private Angle currentGoal = Degrees.of(0);

  private enum ArmState {
    TOP(Degrees.of(0)),
    BOTTOM(Degrees.of(0));

    private Angle setpoint = Degrees.of(0);

    ArmState(Angle setpoint) {
      this.setpoint = setpoint;
    }

    public Angle getSetpoint() {
      return setpoint;
    }
  }

  public static final Angle AT_SETPOINT_TOLERANCE = Degrees.of(2.0);

  public static final Angle ARM_MIN_ANGLE = Degrees.of(-360.0);
  public static final Angle ARM_MAX_ANGLE = Degrees.of(360.0);
  public static final Angle ARM_STARTING_ANGLE = Degrees.of(0.0);

  @Getter
  @AutoLogOutput(key = "Arm/AtGoal")
  private boolean atGoal = false;

  @AutoLogOutput @Getter private boolean homed = false;

  private Debouncer goalDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

  public Arm(ArmIO io) {
    this.io = io;
    io.resetPosition(Degrees.of(0));
  }

  public void goToPosition(ArmState armState) {
    setpoint = armState.getSetpoint();
    // if (setpoint instanceof Angle angleSetpoint) {
    Angle clampedSetpoint = setpoint;
    if (setpoint.lt(ARM_MIN_ANGLE)) {
      clampedSetpoint = ARM_MIN_ANGLE;
    } else if (setpoint.gt(ARM_MAX_ANGLE)) {
      clampedSetpoint = ARM_MAX_ANGLE;
    }

    this.setpoint = clampedSetpoint;
    this.currentGoal = clampedSetpoint;
    this.atGoal = false;
    goalDebouncer.calculate(false);
    Logger.recordOutput("Arm/CommandedSetpoint", clampedSetpoint.in(Degrees));
    /*} else {
    DriverStation.reportError(
        "["
            + "Arm"
            + "] Invalid setpoint type. Expected Angle, got "
            + setpoint.getClass().getSimpleName(),
        false);
        */
    // }
  }

  public Measure<AngleUnit> getCurrentPosition() {
    return inputs.position;
  }

  public void stop() {
    io.stop();
    // Update setpoint to reflect current position when stopped externally
    this.setpoint = inputs.position;
    this.currentGoal = inputs.position;
    this.atGoal = false; // Reset atGoal when stopped
    goalDebouncer.calculate(false); // Reset debouncer state
    Logger.recordOutput("Arm/Stopped", true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.runVolts(Volts.of(percent * 12.0)), () -> io.runVolts(Volts.of(0.0)));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.runVolts(Volts.of((forward.getAsDouble() - reverse.getAsDouble()) * 12.0)),
        () -> io.runVolts(Volts.of(0.0)));
  }

  public Command setSetpointCommand(ArmState targetSetpoint) {
    return Commands.runOnce(() -> goToPosition(targetSetpoint))
        .withName("ArmInternalSet_" + String.format("%.1f", targetSetpoint));
  }
}
