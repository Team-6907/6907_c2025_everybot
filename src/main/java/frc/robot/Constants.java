// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 1; // TODO: Change this
    public static final int LEFT_FOLLOWER_ID = 2; // TODO: Change this
    public static final int RIGHT_LEADER_ID = 3; // TODO: Change this
    public static final int RIGHT_FOLLOWER_ID = 4; // TODO: Change this

    public static final int LEFT_ENCODER_ID = 29; // TODO: Change this
    public static final int RIGHT_ENCODER_ID = 30; // TODO: Change this

    public static final double SLOW_MODE_MOVE = 0.5;
    public static final double SLOW_MODE_TURN = 0.6;

    public static final double FULL_SPEED = 3;

    public static final double KP = 0.3;
    public static final double KI = 0;
    public static final double KD = 0;
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 5;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_CORAL_OUT = -.4;
    public static final double ROLLER_ALGAE_IN = -0.8;
    public static final double ROLLER_ALGAE_OUT = 0.4;
    public static final double ROLLER_CORAL_STACK = -1;
  }

  public static final class ArmConstants {
    public static final int ARM_MOTOR_ID = 6;
    public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
    public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
    public static final double ARM_SPEED_DOWN = 0.4;
    public static final double ARM_SPEED_UP = -0.4;
    public static final double ARM_HOLD_DOWN = 0.1;
    public static final double ARM_HOLD_UP = -0.15;
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 7;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 60;
    public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
    public static final double CLIMBER_SPEED_DOWN = -0.5;
    public static final double CLIMBER_SPEED_UP = 0.5;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
}
