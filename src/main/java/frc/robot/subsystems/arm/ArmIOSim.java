package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ArmIOSim implements ArmIO {
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, armmotorReduction),
          DCMotor.getCIM(1));

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
