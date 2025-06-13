// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

/** This arm implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60. */
public class ArmIOTalonFX implements ArmIO {
  private final TalonFX arm = new TalonFX(armCanId);

  private MotionMagicVoltage mMotionMagicVoltage = new MotionMagicVoltage(0.0);

  private TalonFXConfiguration armConfig = new TalonFXConfiguration();

  private final StatusSignal<Angle> position = arm.getPosition();
  private final StatusSignal<AngularVelocity> velocity = arm.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = arm.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = arm.getSupplyCurrent();

  public ArmIOTalonFX() {
    armConfig.MotionMagic.MotionMagicCruiseVelocity =
        kArmCruiseVelocity.get() / ArmConfigConstants.kArmGearRatio;
    armConfig.MotionMagic.MotionMagicAcceleration =
        kArmAcceleration.get() * armConfig.MotionMagic.MotionMagicCruiseVelocity;
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    armConfig.Slot0.kP = kArmKp.get();
    armConfig.Slot0.kI = kArmKi.get();
    armConfig.Slot0.kD = kArmKd.get();
    armConfig.Slot0.kG = kArmKg.get();
    armConfig.Slot0.kS = kArmKs.get();
    armConfig.Feedback.SensorToMechanismRatio = ArmConfigConstants.kArmGearRatio;
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    arm.getConfigurator().apply(armConfig, 0.25);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, currentAmps);
    arm.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (Constants.tuningMode
        && (kArmKp.hasChanged(hashCode())
            || kArmKd.hasChanged(hashCode())
            || kArmKi.hasChanged(hashCode())
            || kArmKg.hasChanged(hashCode())
            || kArmCruiseVelocity.hasChanged(hashCode())
            || kArmAcceleration.hasChanged(hashCode()))) {
      armConfig.Slot0.kP = kArmKp.get();
      armConfig.Slot0.kI = kArmKi.get();
      armConfig.Slot0.kD = kArmKd.get();
      armConfig.Slot0.kG = kArmKg.get();
      armConfig.MotionMagic.MotionMagicCruiseVelocity =
          kArmCruiseVelocity.get() / ArmConfigConstants.kArmGearRatio;
      armConfig.MotionMagic.MotionMagicAcceleration =
          kArmAcceleration.get() * armConfig.MotionMagic.MotionMagicCruiseVelocity;
      tryUntilOk(5, () -> arm.getConfigurator().apply(armConfig, 0.25));
    }
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, currentAmps);

    inputs.position.mut_replace(position.getValue().in(Rotations), Rotations);
    inputs.velocity.mut_replace(velocity.getValue().in(RotationsPerSecond), RotationsPerSecond);
    inputs.appliedVolts.mut_replace(appliedVolts.getValue());
    inputs.currentAmps.mut_replace(currentAmps.getValue());
  }

  @Override
  public void runSetpoint(Angle position) {
    arm.setControl(mMotionMagicVoltage.withPosition(position));
  }

  @Override
  public void runVolts(Voltage voltage) {
    arm.setVoltage(voltage.in(Volts));
  }

  @Override
  public void resetPosition() {
    arm.setPosition(Rotations.of(-0.03));
    arm.setControl(mMotionMagicVoltage.withPosition(-0.03));
  }

  @Override
  public void stop() {
    arm.stopMotor();
  }
}
