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

package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.arm.ArmConstants;

/**
 * This climber implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX climber = new TalonFX(climberCanId);
  private final StatusSignal<Angle> positionRot = climber.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = climber.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = climber.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = climber.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private MotionMagicVoltage mMotionMagicVoltage = new MotionMagicVoltage(0.0);

  public ClimberIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;    
    config.MotionMagic.MotionMagicCruiseVelocity = 1000 / ClimberConstants.motorReduction;
    config.MotionMagic.MotionMagicAcceleration =
        5000 * config.MotionMagic.MotionMagicCruiseVelocity;


    tryUntilOk(5, () -> climber.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    climber.optimizeBusUtilization();

    config.Slot0.kP = ClimberConstants.real_kP;
    config.Slot0.kI = ClimberConstants.real_kI;
    config.Slot0.kD = ClimberConstants.real_kD;
    config.Slot0.kA = ClimberConstants.real_kA;
    config.Slot0.kS = ClimberConstants.real_kS;
    config.Slot0.kG = ClimberConstants.real_kG;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    climber.setControl(voltageRequest.withOutput(volts));
  }

  public void climbtest(){
    double current_position = climber.getPosition().getValueAsDouble();
    climber.setControl(mMotionMagicVoltage.withPosition(current_position + 1*ClimberConstants.motorReduction));
  }

}
