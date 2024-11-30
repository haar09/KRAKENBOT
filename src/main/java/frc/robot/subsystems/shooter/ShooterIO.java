package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public boolean leftMotorConnected = true;
        public boolean rightMotorConnected = true;
    
        public double leftVelocityRps = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftSupplyCurrentAmps = 0.0;
        public double leftTempCelsius = 0.0;
    
        public double rightVelocityRps = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightSupplyCurrentAmps = 0.0;
        public double rightTempCelsius = 0.0;
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    public void setVelocity(double leftRPS, double rightRPS);
    
    public void neutralMotors();

    public void setSysIdVoltage(Measure<Voltage> volts);
}
