package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    public void setOutputPercentage(double percentage);
    public void stop();

    default void updateInputs(ClimbIOInputs inputs) {}

    @AutoLog
    class ClimbIOInputs {
        public boolean motorConnected = true;

        public double velocityRotationsPerMinute = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelcius = 0.0;
    }
    
}