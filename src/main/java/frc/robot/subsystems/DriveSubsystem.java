// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final WPI_TalonSRX leftLeader;
  private final WPI_TalonSRX leftFollower;
  private final WPI_VictorSPX rightLeader;
  private final WPI_VictorSPX rightFollower;

  private final CANcoder leftEncoder;
  private final CANcoder rightEncoder;

  private final PIDController pidController = new PIDController(DriveConstants.KP, DriveConstants.KI, DriveConstants.KD);

  /**
   * The subsystem used to drive the robot.
   */
  public DriveSubsystem() {
    // create motor controllers for drive
    leftLeader = new WPI_TalonSRX(DriveConstants.LEFT_LEADER_ID);
    leftFollower = new WPI_TalonSRX(DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new WPI_VictorSPX(DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new WPI_VictorSPX(DriveConstants.RIGHT_FOLLOWER_ID);

    //create cancoders for pid
    leftEncoder = new CANcoder(29);
    rightEncoder = new CANcoder(30);

    //set up follow relationships
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
  }

  @Override
  public void periodic() {
    //set up smartdashboard display for encoder values
    SmartDashboard.putNumber("leftEncoder Value", leftEncoder.getPositionSinceBoot().getValueAsDouble());
    SmartDashboard.putNumber("rightEncoderValue", rightEncoder.getPositionSinceBoot().getValueAsDouble());
    SmartDashboard.putNumber("leftEncoder Speed", leftEncoder.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("rightEncoder Speed", rightEncoder.getVelocity().getValueAsDouble());
  }

  /**
   *  Use this to control your drive train, with one axis of the controller moving the robot
   *  forwards and backwards with the other axis turning the robot.
   * 
   * @param xaxisSpeed the speed going forward
   * @param zaxisRotate the speed turning
   */
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    leftLeader.set(xaxisSpeed - zaxisRotate);
    rightLeader.set(- xaxisSpeed - zaxisRotate);
  }

  /**
   *  Use this to control your drive train, with one axis of the controller moving the robot
   *  forwards and backwards with the other axis turning the robot.
   * 
   *  Additionally we use a pidcontroller to make the robot move as your axises input.
   * 
   * @param xaxisSpeed the speed going forward
   * @param zaxisRotate the speed turning
   */

  public void arcadeDrivePID(double xaxisSpeed, double zaxisRotate) {
    leftLeader.set(pidController.calculate(
      leftEncoder.getVelocity().getValueAsDouble()
      , (xaxisSpeed - zaxisRotate)*DriveConstants.FULL_SPEED));
    rightLeader.set(pidController.calculate(
      rightEncoder.getVelocity().getValueAsDouble()
      , (-xaxisSpeed - zaxisRotate)*DriveConstants.FULL_SPEED));
  }
}