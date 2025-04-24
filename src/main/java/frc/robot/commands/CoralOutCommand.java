// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.RollerSubsystem;

/** A CoralOutCommand that uses a roller subsystem. */
public class CoralOutCommand extends Command {
  private final RollerSubsystem m_roller;

  /**
   * Use to score coral into L1.
   *
   * @param roller The subsystem used by this command.
   */
  public CoralOutCommand(RollerSubsystem roller) {
    m_roller = roller;
    addRequirements(roller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_roller.runRoller(RollerConstants.ROLLER_CORAL_OUT);
  }

  // Called once the command ends or is interrupted. Ensures the roller
  // is not running after we let go of the button.
  @Override
  public void end(boolean interrupted) {
    m_roller.runRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
