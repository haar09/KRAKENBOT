package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.NEO;

public class ShooterPivot extends SubsystemBase{
    
    private final NEO pivotMotor;

    private final CANcoder absoluteEncoder;

    private final PIDController anglePidController;

    public ShooterPivot() {
        pivotMotor = new NEO(ShooterConstants.kPivotMotorId, ShooterConstants.kPivotMotorReversed, IdleMode.kBrake);
        absoluteEncoder = new CANcoder(ShooterConstants.kAbsoluteEncoderId);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = ShooterConstants.kAbsoluteEncoderOffset;
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(config);

        anglePidController = new PIDController(ShooterConstants.kAngleP, ShooterConstants.kAngleI, ShooterConstants.kAngleD);
        anglePidController.setTolerance(ShooterConstants.kAngleToleranceRad);
        anglePidController.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putNumber("customAngle", VisionConstants.y_ArmAngle[0]);

        resetEncoders();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Angle", Math.toDegrees(getAngle()));
    }

    public double getAngle(){
        return absoluteEncoder.getPosition().getValueAsDouble() * ShooterConstants.kPivotMotorRot2Rad;
    }

    public double getAbsolutePosition(){
        return (absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    public void resetEncoders(){
        absoluteEncoder.setPosition(getAbsolutePosition() / 360);
    }

    public void setDesiredAngle(double angle){
        angle = Math.toRadians(angle);
        if (Math.abs(angle) < ShooterConstants.kAngleToleranceRad) {
            stop();
            return;
        }

        if (angle > ShooterConstants.kMaxShooterAngle) {
            angle = ShooterConstants.kMaxShooterAngle;
        } else if (angle < ShooterConstants.kMinShooterAngle) {
            angle = ShooterConstants.kMinShooterAngle;
        }

        pivotMotor.set(anglePidController.calculate(getAngle(), angle));
    }
    
    public void stop(){
        pivotMotor.set(0);
    }
}