// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;

public class AlgaeInCommand extends Command {
  private final Roller m_roller;

  public AlgaeInCommand(Roller roller) {
    m_roller = roller;
    addRequirements(roller);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_roller.runPercent(RollerConstants.ROLLER_ALGAE_OUT_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    m_roller.runPercent(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
