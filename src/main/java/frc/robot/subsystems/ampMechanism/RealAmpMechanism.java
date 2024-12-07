package frc.robot.subsystems.ampMechanism;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.ClimbConstants;

public class RealAmpMechanism implements AmpMechanismIO{
    private TalonSRX ampMotor;

    public RealAmpMechanism() {
        ampMotor = new TalonSRX(ClimbConstants.kAmpMechanismMotorId);

        ampMotor.setInverted(ClimbConstants.kAmpMechanismMotorReversed);

        ampMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void setOutputPercentage(double percentage) {
        ampMotor.set(TalonSRXControlMode.PercentOutput, percentage * ClimbConstants.kAmpMechanismMotorMultiplier);
    }

    @Override
    public void stop(){
        ampMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    @Override
    public void updateInputs(AmpMechanismIOInputs inputs){
        inputs.motorConnected = ampMotor.getBusVoltage() > 0.0;

        inputs.outputPercentage = ampMotor.getMotorOutputPercent();
        inputs.appliedVolts = ampMotor.getMotorOutputVoltage();
        inputs.supplyCurrentAmps = ampMotor.getSupplyCurrent();
        inputs.tempCelcius = ampMotor.getTemperature();
    }
}
