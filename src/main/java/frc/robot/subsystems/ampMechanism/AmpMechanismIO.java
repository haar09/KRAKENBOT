package frc.robot.subsystems.ampMechanism;

import org.littletonrobotics.junction.AutoLog;

public interface AmpMechanismIO {
    public void setOutputPercentage(double percentage);
    public void stop();

    default void updateInputs(AmpMechanismIOInputs inputs) {}

    @AutoLog
    class AmpMechanismIOInputs {
        public boolean motorConnected = true;

        public double outputPercentage = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelcius = 0.0;
    }
    
}