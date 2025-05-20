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

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.PhoenixUtil.*;


import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;

/** This drive implementation is for Talon FXs driving motors like the Falon 500 or Kraken X60. */
public class DriveIOTalonSRXandVictorSPX implements DriveIO {
  private static final double tickPerRevolution = 1440;

  private final TalonSRX leftLeader = new TalonSRX(leftLeaderCanId);
  private final VictorSPX leftFollower = new VictorSPX(leftFollowerCanId);
  private final TalonSRX rightLeader = new TalonSRX(rightLeaderCanId);
  private final VictorSPX rightFollower = new VictorSPX(rightFollowerCanId);

  private final CANcoder leftCANcoder = new CANcoder(1);
  private final CANcoder rightCANcoder = new CANcoder(2);

  public DriveIOTalonSRXandVictorSPX() {
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    //config TalonSRX
    var configTalon = new TalonSRXConfiguration();
    configTalon.peakCurrentLimit = currentLimit;
    configTalon.peakCurrentDuration = 250;
    configTalon.voltageCompSaturation = 12.0;

    tryUntilOkV5(5, () -> leftLeader.configAllSettings(configTalon));
    tryUntilOkV5(5, () -> rightLeader.configAllSettings(configTalon));

    leftLeader.setInverted(leftInverted);
    rightLeader.setInverted(rightInverted);

    //config VictorSPX
    //nothing to config

  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionRad =
        Units.rotationsToRadians(leftCANcoder.getPosition().getValueAsDouble() * tickPerRevolution);
    inputs.leftVelocityRadPerSec =
        Units.rotationsToRadians(
            leftLeader.getSelectedSensorVelocity()
                / tickPerRevolution
                * 10.0); // Raw units are ticks per 100ms :(
    inputs.leftAppliedVolts = leftLeader.getMotorOutputVoltage();
    inputs.leftCurrentAmps =
        new double[] {leftLeader.getStatorCurrent()};

    inputs.rightPositionRad =
        Units.rotationsToRadians(rightCANcoder.getPosition().getValueAsDouble() * tickPerRevolution);
    inputs.rightVelocityRadPerSec =
        Units.rotationsToRadians(
            rightLeader.getSelectedSensorVelocity()
                / tickPerRevolution
                * 10.0); // Raw units are ticks per 100ms :(
    inputs.rightAppliedVolts = rightLeader.getMotorOutputVoltage();
    inputs.rightCurrentAmps =
        new double[] {rightLeader.getStatorCurrent()};
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.set(TalonSRXControlMode.PercentOutput, leftVolts / 12.0);
    rightLeader.set(TalonSRXControlMode.PercentOutput, rightVolts / 12.0);
  }

  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    // OK to just divide FF by 12 because voltage compensation is enabled
    leftLeader.set(
        TalonSRXControlMode.Velocity,
        Units.radiansToRotations(leftRadPerSec) / 10.0, // Raw units are ticks per 100ms :(
        DemandType.ArbitraryFeedForward,
        leftFFVolts / 12.0);
    rightLeader.set(
        TalonSRXControlMode.Velocity,
        Units.radiansToRotations(rightRadPerSec) / 10.0, // Raw units are ticks per 100ms :(
        DemandType.ArbitraryFeedForward,
        rightFFVolts / 12.0);
  }
}
