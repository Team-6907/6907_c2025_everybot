// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.Armposition;

public class ArmDownCommand extends Command {
  private final Arm m_arm;

  public ArmDownCommand(Arm arm) {
    m_arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_arm.runtoPose(Armposition.BOTTOM);
  }

  // Called once the command ends or is interrupted.
  // Here we run a command that will hold the arm up after to ensure the arm does
  // not drop due to gravity.
  @Override
  public void end(boolean interrupted) {
    m_arm.runPercent(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
