package frc.robot.subsystems.rollers.intake;

import org.littletonrobotics.junction.Logger;

import frc.robot.Robot;

public class Intake{
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static Intake create() {
        return new Intake(Robot.isReal() ? new RealIntake() : new SimIntake());
    }

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void setOutputPercentage(double percentage) {
        io.setOutputPercentage(percentage);
    }

    public void stop(){
        io.setOutputPercentage(0);
    }
}
