package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.Armposition;

public class ArmCommands {

  private ArmCommands() {}

  public static Command ArmTOPCommand(Arm arm) {
    return arm.runtoPose(Armposition.TOP);
  }

  public static Command ArmBOTTOMCommand(Arm arm) {
    return arm.runtoPose(Armposition.BOTTOM);
  }
}
