// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.RollerSubsystem;

/**
 * A CoralStackCommand that uses a roller subsystem. Used when coral is already in the L1 to get it
 * past the first once.
 */
public class CoralStackCommand extends Command {
  private final RollerSubsystem m_roller;

  /**
   * Use this command when there is already coral in L1.
   *
   * <p>May be less reliable if there is no coral already in L1.
   *
   * @param roller The subsystem used by this command.
   */
  public CoralStackCommand(RollerSubsystem roller) {
    m_roller = roller;
    addRequirements(roller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_roller.runRoller(RollerConstants.ROLLER_CORAL_STACK);
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
