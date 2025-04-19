// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// Command to drive the robot with joystick inputs
public class DriveCommand extends Command {
  private final DoubleSupplier m_xSpeed;
  private final DoubleSupplier m_zRotation;
  private final DriveSubsystem m_drive;
  private final BooleanSupplier m_squared;

  /**
   * Used to drive the robot, uses arcade drive by default, you will need to modify
   * this command to use tank if desired.
   * 
   * @param driveSubsystem 
   * @param xSpeed The speed fowards and backwards
   * @param zRotation The speed to turn the drivetrain at
   * @param squareInputs Square the inputs from the controller
   */
  public DriveCommand(DriveSubsystem driveSubsystem, 
      DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier squareInputs) {
    // Save parameters to local variables for use later
    m_xSpeed = xSpeed;
    m_zRotation = zRotation;
    m_drive = driveSubsystem;
    m_squared = squareInputs;

    // Declare subsystems required by this command. This prevents the 
    // subsystem from being called by another command while this command is being used.
    addRequirements(m_drive);
  }

  // Runs each time the command is scheduled.
  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  // In teleop we square the drive command to help improve hanlding, play
  // around with it off, this is driver preference
  @Override
  public void execute() {
    m_drive.driveArcade(m_xSpeed.getAsDouble(), m_zRotation.getAsDouble(), m_squared.getAsBoolean());
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // Return false to indicate that this command never ends. It can be interrupted
    // by another command needing the same subsystem.
    return false;
  }
}
