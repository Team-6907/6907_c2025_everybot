package frc.robot.subsystems.arm;

public class ArmConstants {
  public static final int armCanId = 9;
  public static final double armmotorReduction = 23;
  public static final int currentLimit = 40;

  public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
  public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
  public static final double ARM_SPEED_DOWN = 0.4;
  public static final double ARM_SPEED_UP = -0.4;
  public static final double ARM_HOLD_DOWN = 0;
  public static final double ARM_HOLD_UP = -0;
  public static final double ARM_POSITION_TOP = 0; // degree  need to change, when arm is home
  public static final double ARM_POSITION_BOTTOM = 90; // degree  need to change

  public static final double real_kP = 0;
  public static final double real_kI = 0;
  public static final double real_kD = 0;
  public static final double real_kA = 0;
  public static final double real_kS = 0;
  public static final double real_kG = 0;

  public static final double sim_kP = 0;
  public static final double sim_kI = 0;
  public static final double sim_kD = 0;

  public static final double kArmGearRatio = 36 / 24;
}
