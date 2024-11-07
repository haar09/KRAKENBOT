package frc.robot.subsystems.extender;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.IntakextenderConstants;
import frc.robot.util.NEO;

public class RealExtender implements ExtenderIO{
    private final NEO extenderMotor = new NEO(IntakextenderConstants.kExtenderMotorId, IntakextenderConstants.kExtenderMotorReversed, IdleMode.kBrake);
    
    public RealExtender() {
    }

    @Override
    public void setOutputPercentage(double percentage) {
        extenderMotor.set(percentage);
    }

    @Override
    public void stop(){
        extenderMotor.set(0);
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs){
        inputs.motorConnected = extenderMotor.isConnected();

        inputs.velocityRotationsPerMinute = extenderMotor.getVelocity();
        inputs.appliedVolts = extenderMotor.getAppliedVoltage();
        inputs.supplyCurrentAmps = extenderMotor.getCurrent();
        inputs.tempCelcius = extenderMotor.getTemperature();
    }
}
