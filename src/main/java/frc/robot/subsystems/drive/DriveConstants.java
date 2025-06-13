// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants.*;
import frc.robot.util.LoggedTunableNumber;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.0;
  public static final double trackWidth = Units.inchesToMeters(26.0);

  // Device CAN IDs
  public static final int pigeonCanId = 50; // TODO: Change this.
  public static final int leftLeaderCanId = 0;
  public static final int leftFollowerCanId = 1;
  public static final int rightLeaderCanId = 2;
  public static final int rightFollowerCanId = 3;
  public static final int leftCANcoderCanId = 29;
  public static final int rightCANcoderCanId = 30;

  public static final double kDriveGearRatio = 1 / 8.46;
  public static final double kWheelDiameter = 6.0 * 2.54 / 100.0;
  // Motor configuration
  public static final int currentLimit = 60;
  public static final double wheelRadiusMeters = Units.inchesToMeters(3.0);
  public static final double motorReduction = 8.46;
  public static final boolean leftInverted = false;
  public static final boolean rightInverted = true;
  public static final DCMotor gearbox = DCMotor.getCIM(2);

  // Velocity PID configuration
  public static final LoggedTunableNumber realKp = new LoggedTunableNumber("Drive/Kp", 2.5);
  public static final LoggedTunableNumber realKi = new LoggedTunableNumber("Drive/Ki", 0.0);
  public static final LoggedTunableNumber realKd = new LoggedTunableNumber("Drive/Kd", 0.0);

  public static final double realKs = 0.0; // TODO: Change this.
  public static final double realKv = 0.1; // TODO: Change this.

  public static final double simKp = 0.05;
  public static final double simKd = 0.0;
  public static final double simKs = 0.0;
  public static final double simKv = 0.227;

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              gearbox.withReduction(motorReduction),
              currentLimit,
              2),
          trackWidth);
}
