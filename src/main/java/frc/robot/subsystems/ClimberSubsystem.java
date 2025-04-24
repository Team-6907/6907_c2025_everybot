package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX climberMotorTalonFX;

    public ClimberSubsystem () {
    climberMotorTalonFX = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    //climbMotor.setCANTimeout(250);

    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    climberConfig.Voltage.PeakForwardVoltage = 12.0;
    climberConfig.Voltage.PeakReverseVoltage = -12.0;
    climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climberConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    climberMotorTalonFX.getConfigurator().apply(climberConfig, Constants.kLongCANTimeoutSec);

    }

    @Override
    public void periodic() {
    }

    /**
     * Use to run the climber, can be set to run from 100% to -100%.
     * Keep in mind that the direction changes based on which way the winch is wound.
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runClimber(double speed){
        climberMotorTalonFX.set(speed);
    }

}