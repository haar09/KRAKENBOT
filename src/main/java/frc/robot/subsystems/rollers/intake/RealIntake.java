package frc.robot.subsystems.rollers.intake;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.IntakextenderConstants;
import frc.robot.util.NEO;

public class RealIntake implements IntakeIO{
    private final NEO intakeMotor = new NEO(IntakextenderConstants.kIntakeMotorId, IntakextenderConstants.kIntakeMotorReversed, IdleMode.kCoast);
    
    public RealIntake() {
    }

    @Override
    public void setOutputPercentage(double percentage) {
        intakeMotor.set(percentage);
    }

    @Override
    public void stop(){
        intakeMotor.set(0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.motorConnected = intakeMotor.isConnected();

        inputs.velocityRotationsPerMinute = intakeMotor.getVelocity();
        inputs.appliedVolts = intakeMotor.getAppliedVoltage();
        inputs.supplyCurrentAmps = intakeMotor.getCurrent();
        inputs.tempCelcius = intakeMotor.getTemperature();
    }
}
