package frc.robot.commands;

import javax.sound.midi.Sequencer;

import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Armposition;
import frc.robot.subsystems.RollerSubsystem;

public class AlgaeGroundCommand extends Command {

  private final RollerSubsystem m_roller;
  private final ArmSubsystem m_arm;
  public final Armposition m_p = Armposition.TOP;

  public AlgaeGroundCommand(RollerSubsystem roller, ArmSubsystem arm) {
      m_roller = roller;
      m_arm = arm;
      addRequirements(arm);
      addRequirements(roller);
      
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    new SequentialCommandGroup(
      new ArmMovePosCommand(m_arm, Armposition.BOTTOM),
      new AlgieInCommand(m_roller)
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
