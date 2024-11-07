package frc.robot.subsystems.extender;

import org.littletonrobotics.junction.AutoLog;

public interface ExtenderIO {
    public void setOutputPercentage(double percentage);
    public void stop();

    default void updateInputs(ExtenderIOInputs inputs) {}

    @AutoLog
    class ExtenderIOInputs {
        public boolean motorConnected = true;

        public double velocityRotationsPerMinute = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelcius = 0.0;
    }
    
}