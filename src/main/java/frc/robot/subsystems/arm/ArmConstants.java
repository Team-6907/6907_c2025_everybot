// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.arm;

public class ArmConstants {
  public static final int armCanId = 5; // TODO: Change this.
  public static final double motorReduction = 1.0;
  public static final int currentLimit = 40;

  public static final class ArmConfigConstants {
    public static final int kArmGearRatio = 36 / 24; // TODO: Change this.

    public static final double kArmCurrentLimit = 30.0;

    public static final double kArmForwardLimit = 4.8;
    public static final double kArmReverseLimit = 0.05;
  }
}
