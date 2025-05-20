// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.CoralCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOTalonSRXandVictorSPX;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Roller roller;
  private final Arm arm;
  private final Climber climber;

  // Controller
  // private final CommandXboxController controller = new CommandXboxController(0);

  private final CommandXboxController driverController =
      new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOTalonSRXandVictorSPX(), new GyroIOPigeon2());
        roller = new Roller(new RollerIOTalonFX());
        arm = new Arm(new ArmIOTalonFX());
        climber = new Climber(new ClimberIOTalonFX() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        roller = new Roller(new RollerIOSim());
        arm = new Arm(new ArmIOSim());
        climber = new Climber(new ClimberIOSim() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        roller = new Roller(new RollerIO() {});
        arm = new Arm(new ArmIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "ScoreCoral", CoralCommands.CoralOutCommand(roller).withTimeout(2.0));
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

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default drive command, normal arcade drive

    arm.resetPosion();

    drive.setDefaultCommand(
        DriveCommands.arcadeDrive(
            drive, () -> -driverController.getLeftY(), () -> -driverController.getRightX()));

    driverController
        .leftBumper()
        .whileTrue(
            DriveCommands.arcadeDrive(
                drive,
                () -> -driverController.getLeftY() * DriveConstants.SLOW_MODE_MOVE,
                () -> -driverController.getRightX() * DriveConstants.SLOW_MODE_TURN));

    operatorController.rightBumper().whileTrue(AlgaeCommands.SimpleAlgaeInCommand(roller));
    // operatorController.rightBumper().whileTrue(roller.runPercent(1.0));

    // Here we use a trigger as a button when it is pushed past a certain threshold
    operatorController.rightTrigger(.2).whileTrue(AlgaeCommands.SimpleAlgaeOutCommand(roller));

    operatorController.leftBumper().whileTrue(ArmCommands.ArmTOPCommand(arm));
    operatorController.leftTrigger(.2).whileTrue(ArmCommands.ArmBOTTOMCommand(arm));

    /**
     * Used to score coral, the stack command is for when there is already coral in L1 where you are
     * trying to score. The numbers may need to be tuned, make sure the rollers do not wear on the
     * plastic basket.
     */
    operatorController.x().whileTrue(CoralCommands.CoralOutCommand(roller));
    operatorController.y().whileTrue(CoralCommands.CoralStackCommand(roller));

    /**
     * POV is a direction on the D-Pad or directional arrow pad of the controller, the direction of
     * this will be different depending on how your winch is wound
     */
    operatorController.pov(0).whileTrue(ClimbCommands.ClimbUp(climber));
    operatorController.pov(180).whileTrue(ClimbCommands.ClimbDown(climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
