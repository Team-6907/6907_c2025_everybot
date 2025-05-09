// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.armCanId;
import static frc.robot.subsystems.roller.RollerConstants.currentLimit;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.arm.Arm.Armposition;
import org.littletonrobotics.junction.Logger;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class ArmIOTalonFX implements ArmIO {
  private final TalonFX arm = new TalonFX(armCanId);
  private final StatusSignal<Angle> positionRot = arm.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = arm.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = arm.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = arm.getSupplyCurrent();
  private double goal_setpoint;

  private final PIDController pidController =
      new PIDController(ArmConstants.real_kP, ArmConstants.real_kI, ArmConstants.real_kD);

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ArmIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> arm.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    arm.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    arm.setControl(voltageRequest.withOutput(volts));
  }

  public double getPosition() {
    return arm.getPosition().getValueAsDouble();
  }

  public void ArmtoPose(Armposition p) {
    // switch armpos to double(int)
    switch (p) {
      case BOTTOM:
        goal_setpoint = ArmConstants.ARM_POSITION_BOTTOM;
        break;
      case TOP:
        goal_setpoint = ArmConstants.ARM_POSITION_TOP;
        break;
      default:
        goal_setpoint = ArmConstants.ARM_POSITION_TOP;
    }

    Logger.recordOutput("Arm_goal_setpoint", goal_setpoint);

    pidController.setSetpoint(goal_setpoint);
    double speed = pidController.calculate(arm.getPosition().getValueAsDouble());
    arm.setVoltage(speed);
  }

  public void resetPosition() {
    arm.setPosition(0);
  }
}
