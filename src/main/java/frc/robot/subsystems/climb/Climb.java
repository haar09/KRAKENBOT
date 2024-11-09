package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public static Climb create() {
        return new Climb(Robot.isReal() ? new RealClimb() : new NoClimb());
    }

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public void setOutputPercentage(double percentage) {
        io.setOutputPercentage(percentage);
    }

    public void stop(){
        io.setOutputPercentage(0);
    }
}
