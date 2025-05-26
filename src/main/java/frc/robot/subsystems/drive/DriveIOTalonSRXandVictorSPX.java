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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** This drive implementation is for Talon FXs driving motors like the Falon 500 or Kraken X60. */
public class DriveIOTalonSRXandVictorSPX implements DriveIO {
  private static final double tickPerRevolution = 4096 / motorReduction;

  private final TalonSRX leftLeader = new TalonSRX(leftLeaderCanId);
  private final VictorSPX leftFollower = new VictorSPX(leftFollowerCanId);
  private final TalonSRX rightLeader = new TalonSRX(rightLeaderCanId);
  private final VictorSPX rightFollower = new VictorSPX(rightFollowerCanId);

  private final CANcoder leftCANcoder = new CANcoder(29);
  private final CANcoder rightCANcoder = new CANcoder(30);

  private final ProfiledPIDController leftPIDController =
      new ProfiledPIDController(
          1.0,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(maxSpeedMetersPerSec * motorReduction, 2.0),
          0.02);
  private final ProfiledPIDController rightPIDController =
      new ProfiledPIDController(
          1.0,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(maxSpeedMetersPerSec * motorReduction, 2.0),
          0.02);

  public DriveIOTalonSRXandVictorSPX() {
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // config TalonSRX
    var configTalon = new TalonSRXConfiguration();
    configTalon.peakCurrentLimit = currentLimit;
    configTalon.peakCurrentDuration = 250;
    configTalon.voltageCompSaturation = 12.0;

    tryUntilOkV5(5, () -> rightLeader.configAllSettings(configTalon));
    tryUntilOkV5(5, () -> leftLeader.configAllSettings(configTalon));

    leftLeader.setInverted(leftLeaderInverted);
    rightLeader.setInverted(rightLeaderInverted);
    leftFollower.setInverted(leftFollowerInverted);
    rightFollower.setInverted(rightFollowerInverted);
    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);

    // config VictorSPX
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionRad =
        Units.rotationsToRadians(leftCANcoder.getPosition().getValueAsDouble() / motorReduction);
    inputs.leftVelocityRadPerSec =
        Units.rotationsToRadians(
            leftLeader.getSelectedSensorVelocity() * 10.0 / motorReduction); // Raw units are ticks per 100ms :(
    inputs.leftAppliedVolts = leftLeader.getMotorOutputVoltage();
    inputs.leftCurrentAmps = new double[] {leftLeader.getStatorCurrent()};

    inputs.rightPositionRad =
        Units.rotationsToRadians(rightCANcoder.getPosition().getValueAsDouble() / motorReduction);
    inputs.rightVelocityRadPerSec =
        Units.rotationsToRadians(
            rightLeader.getSelectedSensorVelocity() * 10.0 / motorReduction); // Raw units are ticks per 100ms :(
    inputs.rightAppliedVolts = rightLeader.getMotorOutputVoltage();
    inputs.rightCurrentAmps = new double[] {rightLeader.getStatorCurrent()};
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.set(TalonSRXControlMode.PercentOutput, leftVolts / 12.0);
    rightLeader.set(TalonSRXControlMode.PercentOutput, rightVolts / 12.0);
  }

  @Override
  public void setVelocity(double leftRadPerSec, double rightRadPerSec) {
    // OK to just divide FF by 12 because voltage compensation is enabled

    double m_leftVolt = leftPIDController.calculate(leftRadPerSec);
    double m_rightVolt = rightPIDController.calculate(rightRadPerSec);

    leftLeader.set(ControlMode.PercentOutput, m_leftVolt / 12);
    rightLeader.set(ControlMode.PercentOutput, m_rightVolt / 12);

    /*leftLeader.set(
        TalonSRXControlMode.Velocity,
        Units.radiansToRotations(leftPIDController.calculate(leftRadPerSec)) / 10.0, // Raw units are ticks per 100ms :(
        DemandType.ArbitraryFeedForward,
        leftFFVolts / 12.0);
    rightLeader.set(
        TalonSRXControlMode.Velocity,
        Units.radiansToRotations(leftPIDController.calculate(rightRadPerSec)) / 10.0, // Raw units are ticks per 100ms :(
        DemandType.ArbitraryFeedForward,
        rightFFVolts / 12.0);
        */
  }
}
