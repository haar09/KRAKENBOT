package frc.robot.subsystems.rollers.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    public void setOutputPercentage(double percentage);
    public void stop();

    default void updateInputs(IntakeIOInputs inputs) {}

    @AutoLog
    class IntakeIOInputs {
        public boolean motorConnected = true;

        public double velocityRotationsPerMinute = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelcius = 0.0;
    }
    
}