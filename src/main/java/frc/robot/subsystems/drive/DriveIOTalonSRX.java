// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.*;

/** This drive implementation is for Talon SRXs driving brushed motors (e.g. CIMS) with encoders. */
public class DriveIOTalonSRX implements DriveIO {
  private final TalonSRX leftLeader = new TalonSRX(leftLeaderCanId);
  private final VictorSPX leftFollower = new VictorSPX(leftFollowerCanId);
  private final TalonSRX rightLeader = new TalonSRX(rightLeaderCanId);
  private final VictorSPX rightFollower = new VictorSPX(rightFollowerCanId);

  private final CANcoder leftCANcoder = new CANcoder(leftCANcoderCanId);
  private final CANcoder rightCANcoder = new CANcoder(rightCANcoderCanId);

  private final CANcoderConfiguration leftCANcoderConfig = new CANcoderConfiguration();
  private final CANcoderConfiguration rightCANcoderConfig = new CANcoderConfiguration();

  private final PIDController pidController =
      new PIDController(realKp.get(), realKi.get(), realKd.get());

  private MutLinearVelocity leftVelocityMetPerSec = MetersPerSecond.mutable(0);
  private MutLinearVelocity rightVelocityMetPerSec = MetersPerSecond.mutable(0);

  private Voltage leftAppliedVoltage = Volts.of(0);
  private Voltage rightAppliedVoltage = Volts.of(0);

  public DriveIOTalonSRX() {
    var config = new TalonSRXConfiguration();
    config.peakCurrentLimit = currentLimit;
    config.continuousCurrentLimit = currentLimit - 15;
    config.peakCurrentDuration = 250;
    config.voltageCompSaturation = 12.0;
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;

    tryUntilOkV5(5, () -> leftLeader.configAllSettings(config));
    tryUntilOkV5(5, () -> rightLeader.configAllSettings(config));

    leftLeader.setInverted(true);
    rightLeader.setInverted(false);
    leftFollower.setInverted(true);
    rightFollower.setInverted(false);

    leftCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    rightCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    leftCANcoder.getConfigurator().apply(leftCANcoderConfig);
    rightCANcoder.getConfigurator().apply(rightCANcoderConfig);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    resetPosition();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {

    inputs.leftPositionRad.mut_replace(
        leftCANcoder.getAbsolutePosition().getValue().in(Radians), Radians);
    leftVelocityMetPerSec.mut_replace(
        leftCANcoder.getVelocity().getValue().in(RotationsPerSecond)
            * kDriveGearRatio
            * kWheelDiameter
            * Math.PI,
        MetersPerSecond);
    inputs.leftPositionMeter.mut_replace(
        inputs.leftPositionRad.in(Rotations) * kDriveGearRatio * kWheelDiameter * Math.PI, Meters);
    inputs.leftVelocityRadPerSec.mut_replace(
        leftCANcoder.getVelocity().getValue().in(RadiansPerSecond), RadiansPerSecond);
    inputs.leftAppliedVolts.mut_replace(this.leftAppliedVoltage);

    inputs.rightPositionRad.mut_replace(
        rightCANcoder.getAbsolutePosition().getValue().in(Radians), Radians);
    rightVelocityMetPerSec.mut_replace(
        rightCANcoder.getAbsolutePosition().getValue().in(Rotations)
            * kDriveGearRatio
            * kWheelDiameter
            * Math.PI,
        MetersPerSecond);
    inputs.rightPositionMeter.mut_replace(
        inputs.rightPositionRad.in(Rotations) * kDriveGearRatio * kWheelDiameter * Math.PI, Meters);
    inputs.rightVelocityRadPerSec.mut_replace(
        rightCANcoder.getVelocity().getValue().in(RadiansPerSecond), RadiansPerSecond);
    inputs.rightVelocityMetPerSec.mut_replace(
        inputs.rightVelocityRadPerSec.in(RotationsPerSecond)
            * kDriveGearRatio
            * kWheelDiameter
            * Math.PI,
        MetersPerSecond);
    inputs.rightAppliedVolts.mut_replace(this.rightAppliedVoltage);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    // OK to just divide by 12 because voltage compensation is enabled
    this.leftAppliedVoltage = Volts.of(leftVolts);
    this.rightAppliedVoltage = Volts.of(rightVolts);

    leftLeader.set(TalonSRXControlMode.PercentOutput, leftVolts / 12.0);
    rightLeader.set(TalonSRXControlMode.PercentOutput, rightVolts / 12.0);
  }

  @Override
  public void setVelocity(LinearVelocity leftMetPerSec, LinearVelocity rightMetPerSec) {
    // OK to just divide FF by 12 because voltage compensation is enabled
    this.leftAppliedVoltage =
        Volts.of(
            pidController.calculate(
                this.leftVelocityMetPerSec.in(MetersPerSecond), leftMetPerSec.in(MetersPerSecond)));
    this.rightAppliedVoltage =
        Volts.of(
            pidController.calculate(
                this.rightVelocityMetPerSec.in(MetersPerSecond),
                rightMetPerSec.in(MetersPerSecond)));

    leftLeader.set(TalonSRXControlMode.PercentOutput, leftAppliedVoltage.in(Volts) / 12.0);
    rightLeader.set(TalonSRXControlMode.PercentOutput, rightAppliedVoltage.in(Volts) / 12.0);
  }

  @Override
  public void resetPosition() {
    leftCANcoder.setPosition(0);
    rightCANcoder.setPosition(0);
  }

  public void stop() {
    this.leftAppliedVoltage = Volts.of(0);
    this.rightAppliedVoltage = Volts.of(0);
    setVoltage(0, 0);
  }

  public void setPID(double Kp, double Ki, double Kd) {
    pidController.setP(Kp);
    pidController.setI(Ki);
    pidController.setD(Kd);
  }
}
