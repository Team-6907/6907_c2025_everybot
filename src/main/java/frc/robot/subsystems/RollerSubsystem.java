package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RollerConstants;

public class RollerSubsystem extends SubsystemBase {

    private final TalonFX rollerMotorTalonFX;

    public RollerSubsystem () {

    rollerMotorTalonFX = new TalonFX(RollerConstants.ROLLER_MOTOR_ID);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    //rollerMotor.setCANTimeout(250);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.Voltage.PeakForwardVoltage = 12.0;
    rollerConfig.Voltage.PeakReverseVoltage = -12.0;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    rollerMotorTalonFX.getConfigurator().apply(rollerConfig, Constants.kLongCANTimeoutSec);

    }

    @Override
    public void periodic() {
    }

    /**
     *  This is a method that makes the roller spin to your desired speed.
     *  Positive values make it spin forward and negative values spin it in reverse.
     * 
     * @param speedmotor speed from -1.0 to 1, with 0 stopping it
     */
    public void runRoller(double speed){
        rollerMotorTalonFX.set(speed);
    }

}