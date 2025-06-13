// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Arm 子系统用于控制机器人的机械臂。
 */
public class Arm extends SubsystemBase {
  private final ArmIO io; // ArmIO接口，用于与硬件交互
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged(); // 自动记录输入数据

  // 可调参数，用于存储不同位置的目标角度值
  private static LoggedTunableNumber intakePosition =
      new LoggedTunableNumber("Arm/intakePosition", 0);
  private static LoggedTunableNumber outtake_startPosition =
      new LoggedTunableNumber("Arm/outtake_startPosition", 0);
  private static LoggedTunableNumber outtake_stopPosition =
      new LoggedTunableNumber("Arm/outtake_stopPosition", 0);
  private static LoggedTunableNumber stow_nonePosition =
      new LoggedTunableNumber("Arm/stow_nonePosition", 0);
  private static LoggedTunableNumber stow_inPosition =
      new LoggedTunableNumber("Arm/stow_inPosition", 0);

  private Angle setPoint = Rotations.of(0.25); // 当前设定点
  private Angle currentGoal = Rotations.of(0.25); // 当前目标角度

  /**
   * ArmState 枚举定义了机械臂的不同状态及其对应的目标角度。
   */
  public enum ArmState {
    INTAKE(Rotations.of(intakePosition.get())), // 采集状态
    OUTTAKE_START(Rotations.of(outtake_startPosition.get())), // 开始释放状态
    OUTTAKE_STOP(Rotations.of(outtake_stopPosition.get())), // 停止释放状态
    STOW_NONE(Rotations.of(stow_nonePosition.get())), // 非存储状态
    STOW_IN(Rotations.of(stow_inPosition.get())); // 存储状态

    private Angle setpoint; // 目标角度

    ArmState(Angle setpoint) {
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
   * 构造函数，初始化Arm子系统。
   *
   * @param io ArmIO接口实例
   */
  public Arm(ArmIO io) {
    this.io = io;
    io.resetPosition(); // 重置位置
  }

  /**
   * 周期性执行的方法，用于更新输入和处理逻辑。
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs); // 更新输入数据

    // 如果处于调优模式且参数已更改，则更新各个状态的角度值
    if (tuningMode
        && (intakePosition.hasChanged(hashCode())
            || outtake_startPosition.hasChanged(hashCode())
            || outtake_stopPosition.hasChanged(hashCode())
            || stow_nonePosition.hasChanged(hashCode())
            || stow_inPosition.hasChanged(hashCode()))) {
      ArmState.INTAKE.updateSetpoint(intakePosition.get());
      ArmState.OUTTAKE_START.updateSetpoint(outtake_startPosition.get());
      ArmState.OUTTAKE_STOP.updateSetpoint(outtake_stopPosition.get());
      ArmState.STOW_NONE.updateSetpoint(stow_nonePosition.get());
      ArmState.STOW_IN.updateSetpoint(stow_inPosition.get());
    }

    // 如果当前目标不等于设定点，则更新目标并记录输出
    if (currentGoal != setPoint) {
      currentGoal = setPoint;
      io.runSetpoint(currentGoal);
      Logger.recordOutput("Arm/CurrentGoal", currentGoal.in(Rotations));
    }

    Logger.processInputs("Arm", inputs); // 处理输入日志
  }

  /**
   * 设置目标状态。
   *
   * @param targetState 目标状态
   */
  public void setSetpoint(ArmState targetState) {
    this.setPoint = targetState.getSetpoint();
    Logger.recordOutput("Arm/Setpoint", setPoint.in(Rotations));
  }

  /**
   * 创建一个命令来设置目标状态。
   *
   * @param targetState 目标状态
   * @return 返回一个Command对象
   */
  public Command setSetpointCommand(ArmState targetState) {
    return Commands.runOnce(() -> setSetpoint(targetState));
  }

  /**
   * 创建一个命令来重置位置。
   *
   * @return 返回一个Command对象
   */
  public Command resetPosition() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              io.stop(); // 停止机械臂
              io.resetPosition(); // 重置位置
            }),
        Commands.runOnce(
            () -> {
              setPoint = Rotations.of(0); // 设定点设为0
              currentGoal = Rotations.of(0); // 当前目标设为0
            }));
  }
}