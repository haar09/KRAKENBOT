package frc.robot.subsystems.rollers.extender;

import org.littletonrobotics.junction.Logger;

import frc.robot.Robot;

public class Extender {
    private final ExtenderIO io;
    private final ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();

    public static Extender create() {
        return new Extender(Robot.isReal() ? new RealExtender() : new NoExtender());
    }

    public Extender(ExtenderIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Extender", inputs);
    }

    public void setOutputPercentage(double percentage) {
        io.setOutputPercentage(percentage);
    }

    public void stop(){
        io.setOutputPercentage(0);
    }
}
