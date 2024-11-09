package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public class NoShooter implements ShooterIO{
    public NoShooter() {
    }

    @Override
    public void neutralMotors() {
    }

    @Override
    public void setSysIdVoltage(Measure<Voltage> volts) {

    }

    @Override
    public void setVelocity(double leftRPS, double rightRPS) {
    }
}
