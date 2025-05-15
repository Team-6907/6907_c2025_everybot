// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CommunicationConstants;
import frc.robot.util.LoggedTunableNumber;

/** This arm implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60. */
public class ArmIOTalonFX implements ArmIO {
  private final TalonFX arm = new TalonFX(armCanId);

  public static final LoggedTunableNumber kArmKp =
      new LoggedTunableNumber("ElevatorConsts/kElevatorKp", 10); // 10
  public static final LoggedTunableNumber kArmKi =
      new LoggedTunableNumber("ElevatorConsts/kElevatorKi", 0.0); // 0
  public static final LoggedTunableNumber kArmKd =
      new LoggedTunableNumber("ElevatorConsts/kElevatorKd", 0.3); // 0.3
  public static final LoggedTunableNumber kArmKg =
      new LoggedTunableNumber("ElevatorConsts/kElevatorKg", 0.2); // 0.25
  public static final LoggedTunableNumber kArmCruiseVelocity =
      new LoggedTunableNumber("ElevatorConsts/kElevatorCruiseVelocity", 45); // 30 with no shooter
  public static final LoggedTunableNumber kArmAcceleration =
      new LoggedTunableNumber("ElevatorConsts/kElevatorAcceleration", 2.5); // 2.0

  private MotionMagicVoltage mMotionMagicVoltage = new MotionMagicVoltage(0.0);

  private TalonFXConfiguration armConfig = new TalonFXConfiguration();

  private final SoftwareLimitSwitchConfigs mSoftLimitConf = new SoftwareLimitSwitchConfigs();

  private final StatusSignal<Angle> position = arm.getPosition();
  private final StatusSignal<AngularVelocity> velocity = arm.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = arm.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = arm.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ArmIOTalonFX() {
    armConfig.MotionMagic.MotionMagicCruiseVelocity =
        kArmCruiseVelocity.get() / ArmConfigConstants.kArmGearRatio; // 30 with no shooter
    armConfig.MotionMagic.MotionMagicAcceleration =
        kArmAcceleration.get()
            * armConfig.MotionMagic.MotionMagicCruiseVelocity; // 30 with no shooter
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = ArmConfigConstants.kArmCurrentLimit;

    armConfig.Slot0.kP = kArmKp.get();
    armConfig.Slot0.kI = kArmKi.get();
    armConfig.Slot0.kD = kArmKd.get();
    armConfig.Slot0.kG = kArmKg.get();
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Feedback.SensorToMechanismRatio = ArmConfigConstants.kArmGearRatio;
    mSoftLimitConf.ForwardSoftLimitThreshold = ArmConfigConstants.kArmForwardLimit;
    mSoftLimitConf.ForwardSoftLimitEnable = true;
    mSoftLimitConf.ReverseSoftLimitThreshold = ArmConfigConstants.kArmReverseLimit;
    mSoftLimitConf.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch = mSoftLimitConf;
    armConfig.Voltage.PeakForwardVoltage = 12.0;
    armConfig.Voltage.PeakReverseVoltage = -12.0;
    armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    armConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = currentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> arm.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, currentAmps);
    arm.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, currentAmps);

    inputs.position.mut_replace(position.getValue().in(Degrees), Degrees);
    inputs.appliedVolts.mut_replace(appliedVolts.getValue());
    inputs.currentAmps.mut_replace(currentAmps.getValue());
  }

  @Override
  public void runVolts(Voltage voltage) {
    arm.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void resetPosition(Angle angle) {
    arm.setPosition(0.0, CommunicationConstants.kLongCANTimeoutSec);
  }
}
