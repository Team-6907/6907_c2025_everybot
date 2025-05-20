package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;

public class AlgaeCommands {

  private AlgaeCommands() {}

  public static Command SimpleAlgaeInCommand(Roller roller) {
    return roller.runPercent(RollerConstants.ROLLER_ALGAE_IN_SPEED);
  }

  public static Command SimpleAlgaeOutCommand(Roller roller) {
    return roller.runPercent(RollerConstants.ROLLER_ALGAE_OUT_SPEED);
  }
}
