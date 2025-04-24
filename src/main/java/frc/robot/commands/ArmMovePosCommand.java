
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Armposition;

public class ArmMovePosCommand extends Command {
  private final ArmSubsystem m_arm;
  private final Armposition m_p;
  private final PIDController pidController;

  public ArmMovePosCommand(ArmSubsystem arm, Armposition p) {

    m_arm = arm;
    m_p = p;
    pidController = new PIDController(3,0,0.8);
    pidController.setSetpoint(m_arm.getSetpoint(m_p));
    addRequirements(arm);
    
  }


  @Override
  public void initialize() {
    System.out.println("ArmMoveCommand started");
    pidController.reset();
    
  }


  @Override
  public void execute() {
    double speed = pidController.calculate(m_arm.getPosition());
    m_arm.runArm(speed);
  }


  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(0);
    System.out.println("ArmMoveCommand ended");
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
