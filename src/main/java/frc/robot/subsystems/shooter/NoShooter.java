package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Voltage;

public class NoShooter implements ShooterIO{
    public NoShooter() {
    }

    @Override
    public void neutralMotors() {
    }

    @Override
    public void setSysIdVoltage(Voltage volts) {

    }

    @Override
    public void setVelocity(double leftRPS, double rightRPS) {
    }
}
