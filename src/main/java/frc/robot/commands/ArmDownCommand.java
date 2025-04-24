// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

/** An ArmDown command that uses an Arm subsystem. */
public class ArmDownCommand extends Command {
  private final ArmSubsystem m_arm;

  /**
   * Powers the arm down, when finished passively holds the arm down.
   *
   * <p>We recommend that you use this to only move the arm into the paracord and let the passive
   * portion hold the arm down.
   *
   * @param arm The subsystem used by this command.
   */
  public ArmDownCommand(ArmSubsystem arm) {
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.runArm(ArmConstants.ARM_SPEED_DOWN);
  }

  // Called once the command ends or is interrupted.
  // Here we run arm down at low speed to ensure it stays down
  // When the next command is caled it will override this command
  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(ArmConstants.ARM_HOLD_DOWN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
