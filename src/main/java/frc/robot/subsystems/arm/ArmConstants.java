// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.arm;

import frc.robot.util.LoggedTunableNumber;

public class ArmConstants {
  public static final int armCanId = 5;

  public static final LoggedTunableNumber kArmKp =
      new LoggedTunableNumber("ArmConstants/kArmKp", 40);
  public static final LoggedTunableNumber kArmKi =
      new LoggedTunableNumber("ArmConstants/kArmKi", 0);
  public static final LoggedTunableNumber kArmKd =
      new LoggedTunableNumber("ArmConstants/kArmKd", 0);
  public static final LoggedTunableNumber kArmKg =
      new LoggedTunableNumber("ArmConstants/kArmKg", 3);
  public static final LoggedTunableNumber kArmKs =
      new LoggedTunableNumber("ArmConstants/kArmKs", 0);
  public static final LoggedTunableNumber kArmCruiseVelocity =
      new LoggedTunableNumber("ArmConstants/kArmCruiseVelocity", 15);
  public static final LoggedTunableNumber kArmAcceleration =
      new LoggedTunableNumber("ArmConstants/kArmAcceleration", 1);

  public static final class ArmConfigConstants {
    public static final double kArmGearRatio = 23 * 24 / 36;
  }
}
