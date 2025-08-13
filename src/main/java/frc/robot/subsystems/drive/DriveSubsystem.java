// Copyright 2021-2025 FRC 6328
package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase {

  private final TalonSRX leftPrimaryMotor;
  private final VictorSPX leftSecondaryMotor;
  private final TalonSRX rightPrimaryMotor;
  private final VictorSPX rightSecondaryMotor;

  private final CANcoder leftEncoder;
  private final CANcoder rightEncoder;

  private final ProfiledPIDController leftPIDController;
  private final ProfiledPIDController rightPIDController;

  private final DifferentialDriveKinematics driveKinematics;
  
  

  public DriveSubsystem() {
   
    leftPrimaryMotor = new TalonSRX(leftLeaderCanId);
    leftSecondaryMotor = new VictorSPX(leftFollowerCanId);
    rightPrimaryMotor = new TalonSRX(rightLeaderCanId);
    rightSecondaryMotor = new VictorSPX(rightFollowerCanId);

    leftSecondaryMotor.follow(leftPrimaryMotor);
    rightSecondaryMotor.follow(rightPrimaryMotor);

    var motorConfig = new TalonSRXConfiguration();
    motorConfig.peakCurrentLimit = currentLimit;
    motorConfig.peakCurrentDuration = 250;
    motorConfig.voltageCompSaturation = 12.0;

    leftPrimaryMotor.configAllSettings(motorConfig);
    rightPrimaryMotor.configAllSettings(motorConfig);

    leftPrimaryMotor.setInverted(leftLeaderInverted);
    rightPrimaryMotor.setInverted(rightLeaderInverted);
    leftSecondaryMotor.setInverted(leftFollowerInverted);
    rightSecondaryMotor.setInverted(rightFollowerInverted);

    setNeutralMode(NeutralMode.Brake);

   
    leftEncoder = new CANcoder(29);
    rightEncoder = new CANcoder(30);


    leftPIDController = new ProfiledPIDController(
        1.0, 0.0, 0.0,
        new TrapezoidProfile.Constraints(maxSpeedMetersPerSec * motorReduction, 2.0),
        0.02);
    rightPIDController = new ProfiledPIDController(
        1.0, 0.0, 0.0,
        new TrapezoidProfile.Constraints(maxSpeedMetersPerSec * motorReduction, 2.0),
        0.02);

    driveKinematics = new DifferentialDriveKinematics(trackWidth);

 
  }

  @Override
  public void periodic() {
    
    Logger.recordOutput("Drive/LeftPosition", getLeftPositionMeters());
    Logger.recordOutput("Drive/RightPosition", getRightPositionMeters());
    Logger.recordOutput("Drive/LeftVelocity", getLeftVelocityMetersPerSec());
    Logger.recordOutput("Drive/RightVelocity", getRightVelocityMetersPerSec());
  }

 
  public void setChassisSpeed(ChassisSpeeds speeds) {
    var wheelSpeeds = driveKinematics.toWheelSpeeds(speeds);
    setWheelSpeeds(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

 
  public void setWheelSpeeds(double leftMetersPerSec, double rightMetersPerSec) {
    double leftRadPerSec = leftMetersPerSec / wheelRadiusMeters;
    double rightRadPerSec = rightMetersPerSec / wheelRadiusMeters;
    
    Logger.recordOutput("Drive/LeftSetpointRadPerSec", leftRadPerSec);
    Logger.recordOutput("Drive/RightSetpointRadPerSec", rightRadPerSec);

    double leftVoltage = leftPIDController.calculate(leftRadPerSec);
    double rightVoltage = rightPIDController.calculate(rightRadPerSec);

    setMotorVoltages(leftVoltage, rightVoltage);
  }


  public void setMotorVoltages(double leftVolts, double rightVolts) {
    leftPrimaryMotor.set(TalonSRXControlMode.PercentOutput, leftVolts / 12.0);
    rightPrimaryMotor.set(TalonSRXControlMode.PercentOutput, rightVolts / 12.0);
  }

 
  public void stop() {
    setMotorVoltages(0.0, 0.0);
  }


  public void setNeutralMode(NeutralMode mode) {
    leftPrimaryMotor.setNeutralMode(mode);
    rightPrimaryMotor.setNeutralMode(mode);
    leftSecondaryMotor.setNeutralMode(mode);
    rightSecondaryMotor.setNeutralMode(mode);
  }




  @AutoLogOutput
  public double getLeftPositionMeters() {
    return Units.rotationsToRadians(leftEncoder.getPosition().getValueAsDouble()) * wheelRadiusMeters;
  }

  @AutoLogOutput
  public double getRightPositionMeters() {
    return Units.rotationsToRadians(rightEncoder.getPosition().getValueAsDouble()) * wheelRadiusMeters;
  }


  @AutoLogOutput
  public double getLeftVelocityMetersPerSec() {
    return Units.rotationsToRadians(leftPrimaryMotor.getSelectedSensorVelocity() * 10.0) * wheelRadiusMeters;
  }

  @AutoLogOutput
  public double getRightVelocityMetersPerSec() {
    return Units.rotationsToRadians(rightPrimaryMotor.getSelectedSensorVelocity() * 10.0) * wheelRadiusMeters;
  }
}