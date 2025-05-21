package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.arm.Arm.Armposition;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createSingleJointedArmSystem(DCMotor.getCIM(1), 0.004, armmotorReduction),
          DCMotor.getCIM(1));

  private double appliedVolts = 0.0;

  private double goal_setpoint;
  private final PIDController pidController =
      new PIDController(ArmConstants.sim_kP, ArmConstants.sim_kI, ArmConstants.sim_kD);

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.positionRad = sim.getAngularPosition().in(Radians);
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  public void ArmtoPose(Armposition p) {
    // switch armpos to double(int)
    switch (p) {
      case BOTTOM:
        goal_setpoint = ArmConstants.ARM_POSITION_BOTTOM;
        break;
      case TOP:
        goal_setpoint = ArmConstants.ARM_POSITION_TOP;
        break;
      default:
        goal_setpoint = ArmConstants.ARM_POSITION_TOP;
    }

    Logger.recordOutput("Arm_goal_setpoint", goal_setpoint);

    pidController.setSetpoint(goal_setpoint);
    double speed = pidController.calculate(sim.getAngularPosition().in(Radians));
    sim.setInputVoltage(speed);
  }

  public void resetPosition() {
    sim.setAngle(0);
  }
}
