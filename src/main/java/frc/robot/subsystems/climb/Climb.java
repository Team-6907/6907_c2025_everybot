// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Climb 控制攀爬机构。
 */
public class Climb extends SubsystemBase {

  private Angle setPoint = Rotations.of(0); // 当前设定角度
  private Angle currentGoal = Rotations.of(0); // 当前目标角度

  private final ClimbIO io; // 硬件接口
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged(); // 自动记录输入数据

  // 可调参数，用于存储不同状态下的角度值
  private static LoggedTunableNumber stowPosition =
      new LoggedTunableNumber("Climb/stowPosition", 0);
  private static LoggedTunableNumber extendedPosition =
      new LoggedTunableNumber("Climb/extendedPosition", 0);
  private static LoggedTunableNumber climbingPosition =
      new LoggedTunableNumber("Climb/climbingPosition", 0);

  /**
   * ClimbState 枚举表示攀爬臂的不同状态及其对应的目标角度。
   */
  public enum ClimbState {
    STOW(Rotations.of(stowPosition.get())), // 存储位置
    EXTENDED(Rotations.of(extendedPosition.get())), // 延伸位置
    CLIMBING(Rotations.of(climbingPosition.get())); // 攀爬位置

    private Angle setpoint; // 目标角度

    ClimbState(Angle setpoint) {
      this.setpoint = setpoint;
    }

    public Angle getSetpoint() {
      return setpoint;
    }

    public void updateSetpoint(double setpoint) {
      this.setpoint = Rotations.of(setpoint);
    }
  }

  /**
   * 构造函数，初始化 Climb 子系统。
   *
   * @param io ClimbIO 接口实例
   */
  public Climb(ClimbIO io) {
    this.io = io;
  }

  /**
   * 周期性执行的方法，用于更新输入和处理逻辑。
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs); // 更新输入数据

    // 如果处于调优模式且参数已更改，则更新各个状态的角度值
    if (tuningMode
        && (stowPosition.hasChanged(hashCode())
            || extendedPosition.hasChanged(hashCode())
            || climbingPosition.hasChanged(hashCode()))) {
      ClimbState.STOW.updateSetpoint(stowPosition.get());
      ClimbState.EXTENDED.updateSetpoint(extendedPosition.get());
      ClimbState.CLIMBING.updateSetpoint(climbingPosition.get());
    }

    Logger.processInputs("Climb", inputs); // 处理日志输入
  }

  /**
   * 创建一个命令以指定百分比功率运行攀爬电机。
   *
   * @param percent 功率百分比（-1 到 1）
   * @return 返回一个Command对象
   */
  public Command runPercent(double percent) {
    return runEnd(
        () -> io.runVolts(Volts.of((percent * 12.0))), () -> io.runVolts(Volts.of((0.0))));
  }

  /**
   * 创建一个命令以根据手动输入控制攀爬电机。
   *
   * @param forward 正向输入
   * @param reverse 反向输入
   * @return 返回一个Command对象
   */
  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.runVolts(Volts.of(((forward.getAsDouble() - reverse.getAsDouble()) * 12.0))),
        () -> io.runVolts(Volts.of((0.0))));
  }

  /**
   * 设置目标状态。
   *
   * @param targetState 目标状态
   */
  public void setSetpoint(ClimbState targetState) {
    this.setPoint = targetState.getSetpoint();
    Logger.recordOutput("Arm/Setpoint", setPoint.in(Rotations));
  }

  /**
   * 创建一个命令来设置目标状态。
   *
   * @param targetState 目标状态
   * @return 返回一个Command对象
   */
  public Command setSetpointCommand(ClimbState targetState) {
    return Commands.runOnce(() -> setSetpoint(targetState));
  }
}