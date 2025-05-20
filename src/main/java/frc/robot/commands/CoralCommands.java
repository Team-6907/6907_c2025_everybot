package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;

public class CoralCommands {

  private CoralCommands() {}

  public static Command CoralOutCommand(Roller roller) {
    return roller.runPercent(RollerConstants.ROLLER_CORAL_OUT);
  }

  public static Command CoralStackCommand(Roller roller) {
    return roller.runPercent(RollerConstants.ROLLER_CORAL_STACK);
  }
}
