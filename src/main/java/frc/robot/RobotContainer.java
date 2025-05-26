// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmState;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOTalonSRX;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Roller roller;
  private final Arm arm;
  private final Climb climb;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOTalonSRX(), new GyroIOPigeon2());
        roller = new Roller(new RollerIOTalonFX());
        arm = new Arm(new ArmIOTalonFX());
        climb = new Climb(new ClimbIO() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        roller = new Roller(new RollerIOSim());
        arm = new Arm(new ArmIOSim());
        climb = new Climb(new ClimbIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        roller = new Roller(new RollerIO() {});
        arm = new Arm(new ArmIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand("Score", roller.runPercent(1.0).withTimeout(3.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.arcadeDrive(
            drive, () -> controller.getLeftY(), () -> controller.getRightX()));

    roller.setDefaultCommand(
        roller.runTeleop(
            () -> controller.getRightTriggerAxis(), () -> controller.getLeftTriggerAxis()));

    controller.a().onTrue(arm.setSetpointCommand(ArmState.STOW_NONE));
    controller.y().onTrue(arm.setSetpointCommand(ArmState.INTAKE));
    controller.rightBumper().onTrue(arm.resetPosition());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
