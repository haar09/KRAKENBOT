package frc.robot.subsystems.pivot;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.NEO;

public class RealPivot implements PivotIO {
    
    private final NEO pivotMotor = new NEO(ShooterConstants.kPivotMotorId, ShooterConstants.kPivotMotorReversed, IdleMode.kBrake);
    private final CANcoder absoluteEncoder = new CANcoder(ShooterConstants.kAbsoluteEncoderId);
    private final PIDController anglePidController = new PIDController(ShooterConstants.kAngleP, ShooterConstants.kAngleI, ShooterConstants.kAngleD);

    public RealPivot() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = ShooterConstants.kAbsoluteEncoderOffset;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(config);

        anglePidController.setTolerance(ShooterConstants.kAngleToleranceRad);
        anglePidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    @Override
    public void setDesiredAngle(double angle){
        angle = Math.toRadians(angle);
        if (Math.abs(angle) < ShooterConstants.kAngleToleranceRad) {
            stop();
            return;
        }

        angle = MathUtil.clamp(angle, ShooterConstants.kMinShooterAngleRad, ShooterConstants.kMaxShooterAngleRad);

        pivotMotor.set(anglePidController.calculate(getAngle(), angle));
    }

    @Override
    public double getAngle(){
        return absoluteEncoder.getPosition().getValueAsDouble() * ShooterConstants.kPivotMotorRot2Rad;
    }

    public double getAbsolutePosition(){
        return (absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    public void resetEncoders(){
        absoluteEncoder.setPosition(getAbsolutePosition() / 360);
    }

    
    @Override
    public void stop(){
        pivotMotor.set(0);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs){
        inputs.motorConnected = pivotMotor.isConnected();
        inputs.absoluteEncoderConnected = BaseStatusSignal.refreshAll(absoluteEncoder.getPosition(), absoluteEncoder.getAbsolutePosition()).isOK();

        inputs.positionRads = getAngle();
        inputs.absoluteEncoderPositionRads = Units.degreesToRadians(getAbsolutePosition());
        inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotMotor.getVelocity());
        inputs.appliedVolts = pivotMotor.getAppliedVoltage();
        inputs.supplyCurrentAmps = pivotMotor.getCurrent();
        inputs.tempCelcius = pivotMotor.getTemperature();
    }

}