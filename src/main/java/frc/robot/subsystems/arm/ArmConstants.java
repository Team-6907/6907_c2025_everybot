// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.arm;

import frc.robot.util.LoggedTunableNumber;

public class ArmConstants {
  public static final int armCanId = 5;
  public static final int currentLimit = 40;

  public static final LoggedTunableNumber kArmKp =
      new LoggedTunableNumber("ArmConstants/kArmKp", 5);
  public static final LoggedTunableNumber kArmKi =
      new LoggedTunableNumber("ArmConstants/kArmKi", 0);
  public static final LoggedTunableNumber kArmKd =
      new LoggedTunableNumber("ArmConstants/kArmKd", 0);
  public static final LoggedTunableNumber kArmKg =
      new LoggedTunableNumber("ArmConstants/kArmKg", 0);
  public static final LoggedTunableNumber kArmKs =
      new LoggedTunableNumber("ArmConstants/kArmKs", 0);
  public static final LoggedTunableNumber kArmCruiseVelocity =
      new LoggedTunableNumber("ArmConstants/kArmCruiseVelocity", 45);
  public static final LoggedTunableNumber kArmAcceleration =
      new LoggedTunableNumber("ArmConstants/kArmAcceleration", 2.5);

  public static final class ArmConfigConstants {
    public static final int kArmGearRatio = 1 / 23 * 36 / 24; // TODO: Change this.

    public static final double kArmCurrentLimit = 40.0;

    public static final double kArmForwardLimit = 12;
    public static final double kArmReverseLimit = 12;
  }
}
