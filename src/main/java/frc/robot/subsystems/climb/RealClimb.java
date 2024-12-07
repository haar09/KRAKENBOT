package frc.robot.subsystems.climb;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimbConstants;
import frc.robot.util.NEO;

public class RealClimb implements ClimbIO{
    private final NEO climbMotor = new NEO(ClimbConstants.kClimbMotorId, ClimbConstants.kClimbMotorReversed, IdleMode.kBrake);
    
    public RealClimb() {
    }

    @Override
    public void setOutputPercentage(double percentage) {
        climbMotor.set(percentage);
    }

    @Override
    public void stop(){
        climbMotor.set(0);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs){
        inputs.motorConnected = climbMotor.isConnected();

        inputs.velocityRotationsPerMinute = climbMotor.getVelocity();
        inputs.appliedVolts = climbMotor.getAppliedVoltage();
        inputs.supplyCurrentAmps = climbMotor.getCurrent();
        inputs.tempCelcius = climbMotor.getTemperature();
    }
}
