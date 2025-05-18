// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.roller.RollerConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class RollerIOTalonFX implements RollerIO {
  private final TalonFX roller = new TalonFX(climbCanId);
  private final StatusSignal<Angle> positionRot = roller.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = roller.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = roller.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = roller.getSupplyCurrent();

  private final MotionMagicVelocityDutyCycle voltageRequest = new MotionMagicVelocityDutyCycle(0);

  public RollerIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> roller.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    roller.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.positionRad.mut_replace(positionRot.getValue().in(Degrees), Degrees);
    inputs.velocityRadPerSec.mut_replace(
        velocityRotPerSec.getValue().in(DegreesPerSecond), DegreesPerSecond);
    inputs.appliedVolts.mut_replace(appliedVolts.getValue());
    inputs.currentAmps.mut_replace(currentAmps.getValue());
  }

  @Override
  public void runVelocity(AngularVelocity angularVelocity) {
    roller.setControl(voltageRequest.withVelocity(angularVelocity));
  }

  @Override
  public void runVolts(Voltage volts) {
    roller.setControl(new VoltageOut(volts));
  }
}
