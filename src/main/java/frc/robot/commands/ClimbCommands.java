package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;

public class ClimbCommands {

  private ClimbCommands() {}

  public static Command ClimbDown(Climber climber) {
    return climber.runPercent(ClimberConstants.CLIMBER_SPEED_DOWN);
  }

  public static Command ClimbUp(Climber climber) {
    return climber.runPercent(ClimberConstants.CLIMBER_SPEED_DOWN);
  }
}
