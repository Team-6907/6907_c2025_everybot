// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ArmIOSim implements ArmIO {
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, motorReduction),
          DCMotor.getCIM(1));

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.position.mut_replace(sim.getAngularPosition().in(Degrees), Degrees);
    inputs.appliedVolts.mut_replace(appliedVolts, Volts);
    inputs.currentAmps.mut_replace(sim.getCurrentDrawAmps(), Amps);
  }

  @Override
  public void runVolts(Voltage voltage) {
    appliedVolts = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
  }
}
