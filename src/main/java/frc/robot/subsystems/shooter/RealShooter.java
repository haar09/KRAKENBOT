package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.*;

public class RealShooter implements ShooterIO{
        private final TalonFX leftMotor = new TalonFX(ShooterConstants.kShooterMotorLeftId);
        private final MotionMagicVelocityVoltage leftMotorVoltageRequest;

        private final TalonFX rightMotor = new TalonFX(ShooterConstants.kShooterMotorRightId);
        private final MotionMagicVelocityVoltage rightMotorVoltageRequest;

        private final BaseStatusSignal
        leftMotorVelocity, rightMotorVelocity,
        leftMotorVoltage, rightMotorVoltage,
        leftMotorTemp, rightMotorTemp,
        leftMotorSupplyCurrent, rightMotorSupplyCurrent;

        public RealShooter() {
            leftMotor.setNeutralMode(NeutralModeValue.Coast);
            rightMotor.setNeutralMode(NeutralModeValue.Coast);

            leftMotor.getConfigurator().apply(ShooterConstants.leftMotorConfig);
            rightMotor.getConfigurator().apply(ShooterConstants.rightMotorConfig);
            leftMotorVoltageRequest = new MotionMagicVelocityVoltage(0);
            rightMotorVoltageRequest = new MotionMagicVelocityVoltage(0);

            leftMotorVelocity = leftMotor.getVelocity();
            rightMotorVelocity = rightMotor.getVelocity();
            leftMotorVoltage = leftMotor.getMotorVoltage();
            rightMotorVoltage = rightMotor.getMotorVoltage();
            leftMotorTemp = leftMotor.getDeviceTemp();
            rightMotorTemp = rightMotor.getDeviceTemp();
            leftMotorSupplyCurrent = leftMotor.getSupplyCurrent();
            rightMotorSupplyCurrent = rightMotor.getSupplyCurrent();

            BaseStatusSignal.setUpdateFrequencyForAll(100,
            leftMotorVelocity,
            leftMotorVoltage,
            rightMotorVelocity,
            rightMotorVoltage);

            BaseStatusSignal.setUpdateFrequencyForAll(250, 
            leftMotorTemp,
            rightMotorTemp,
            leftMotorSupplyCurrent,
            rightMotorSupplyCurrent);

            leftMotor.optimizeBusUtilization();
            rightMotor.optimizeBusUtilization();
        }

        @Override
        public void setVelocity(double left, double right){
            leftMotor.setControl(leftMotorVoltageRequest.withVelocity(left));
            rightMotor.setControl(rightMotorVoltageRequest.withVelocity(-right));
        }

        private final NeutralOut stopRequest = new NeutralOut();

        @Override
        public void neutralMotors() {
            leftMotor.setControl(stopRequest);
            rightMotor.setControl(stopRequest);
        }

        private final VoltageOut m_sysIdControl = new VoltageOut(0);

        public void setSysIdVoltage(Voltage volts) {
            rightMotor.setControl(m_sysIdControl.withOutput(volts.in(Volts)));
        }

        @Override
        public void updateInputs(ShooterIOInputs inputs){
            inputs.leftMotorConnected = BaseStatusSignal.refreshAll(leftMotorVelocity,
            leftMotorVoltage,
            leftMotorTemp,
            leftMotorSupplyCurrent).isOK();
            inputs.rightMotorConnected = BaseStatusSignal.refreshAll(rightMotorVelocity,
            rightMotorVoltage,
            rightMotorTemp,
            rightMotorSupplyCurrent).isOK();;

            inputs.leftVelocityRps = leftMotorVelocity.getValueAsDouble();
            inputs.rightVelocityRps = rightMotorVelocity.getValueAsDouble();

            inputs.leftAppliedVolts = leftMotorVoltage.getValueAsDouble();
            inputs.rightAppliedVolts = rightMotorVoltage.getValueAsDouble();

            inputs.leftSupplyCurrentAmps = leftMotorSupplyCurrent.getValueAsDouble();
            inputs.rightSupplyCurrentAmps = rightMotorSupplyCurrent.getValueAsDouble();

            inputs.leftTempCelsius = leftMotorTemp.getValueAsDouble();
            inputs.rightTempCelsius = rightMotorTemp.getValueAsDouble();
        }
}
