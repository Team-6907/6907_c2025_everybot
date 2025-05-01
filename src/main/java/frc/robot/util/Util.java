// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Util {

  public static void checkReturn(String key, StatusCode code) {
    if (!code.isOK()) {
      DriverStation.reportError(
          String.format("[%s] %s: %s", key, code.getName(), code.getDescription()), null);
    }
  }

  public static String getSubsystemCommand(SubsystemBase subsystem) {
    Command curcmd = subsystem.getCurrentCommand();
    if (curcmd == null) {
      return "[#!烫烫烫!#]";
    } else {
      return curcmd.getName();
    }
  }

  /**
   * Creates a pure translating transform
   *
   * @param x The x coordinate of the translation
   * @param y The y coordinate of the translation
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(double x, double y) {
    return new Transform2d(x, y, new Rotation2d());
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, 1e-9);
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }
}
