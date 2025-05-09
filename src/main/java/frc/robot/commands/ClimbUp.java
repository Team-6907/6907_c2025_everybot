// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;

public class ClimbUp extends Command {
  private final Climber m_climber;


  public ClimbUp(Climber climber) {
    m_climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_climber.runPercent(ClimberConstants.CLIMBER_SPEED_UP);
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.runPercent(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
