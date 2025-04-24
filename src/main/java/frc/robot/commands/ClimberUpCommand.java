// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

/** An ClimberUpCommand that uses a climb subsystem. */
public class ClimberUpCommand extends Command {
  private final ClimberSubsystem m_climber;

  /**
   * Runs the climber up, note that this can change based on how the winch is wound.
   *
   * @param climber The subsystem used by this command.
   */
  public ClimberUpCommand(ClimberSubsystem climber) {
    m_climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.runClimber(ClimberConstants.CLIMBER_SPEED_UP);
  }

  // Called once the command ends or is interrupted.. Here we ensure the climber is not
  // running once we let go of the button
  @Override
  public void end(boolean interrupted) {
    m_climber.runClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
