// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

// Command to drive the robot with joystick inputs
public class DriveCommand extends Command {
  private final DoubleSupplier m_xaxisSpeed;
  private final DoubleSupplier m_zaxisRotation;
  private final DriveSubsystem m_drive;

  /**
   * Used to drive the robot, uses arcadeDrivePID by default.
   * 
   * @param driveSubsystem 
   * @param xaxisSpeed The speed fowards and backwards
   * @param zaxisRotation The speed to turn the drivetrain at
   */
  public DriveCommand(DriveSubsystem driveSubsystem, 
      DoubleSupplier xaxisSpeed, DoubleSupplier zaxisRotation) {
    // Save parameters to local variables for use later
    m_xaxisSpeed = xaxisSpeed;
    m_zaxisRotation = zaxisRotation;
    m_drive = driveSubsystem;

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
    m_drive.arcadeDrivePID(m_xaxisSpeed.getAsDouble(), m_zaxisRotation.getAsDouble());
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
