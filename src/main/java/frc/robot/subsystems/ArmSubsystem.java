package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private final TalonFX armMotorTalonFX;
    
    public ArmSubsystem () {
    armMotorTalonFX = new TalonFX(ArmConstants.ARM_MOTOR_ID);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    //armMotor.setCANTimeout(250);

    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.Voltage.PeakForwardVoltage = 12.0;
    armConfig.Voltage.PeakReverseVoltage = -12.0;
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    armMotorTalonFX.getConfigurator().apply(armConfig, Constants.kLongCANTimeoutSec);
    }

    public enum Armposition {
        BOTTOM,
        TOP,
    }


    @Override
    public void periodic() {

    }

    public double getSetpoint(Armposition p) {
        double setpoint;
        // switch armpos to double(int)
        switch(p) {
            case BOTTOM:
                setpoint = Constants.ArmConstants.ARM_POSITION_BOTTOM;
                break;
            case TOP:
                setpoint = Constants.ArmConstants.ARM_POSITION_TOP;
                break;
            default:
                setpoint = Constants.ArmConstants.ARM_POSITION_TOP;
        }
        return setpoint;
    }

    public void runArm(double speed){
        armMotorTalonFX.set(speed);
    }

    public void resetPosition() {
        armMotorTalonFX.setPosition(0);
    }

    public double getPosition() {
        return armMotorTalonFX.getPosition().getValueAsDouble();
    }

}