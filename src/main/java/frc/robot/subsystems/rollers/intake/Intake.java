package frc.robot.subsystems.rollers.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;

public class Intake{
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final Alert disconnectedAlert;

    public static Intake create() {
        return new Intake(Robot.isReal() ? new RealIntake() : new NoIntake());
    }

    public Intake(IntakeIO io) {
        this.io = io;
        this.disconnectedAlert = new Alert("Intake is disconnected.", AlertType.kWarning);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        disconnectedAlert.set(!inputs.motorConnected);
    }

    public void setOutputPercentage(double percentage) {
        io.setOutputPercentage(percentage);
    }

    public void stop(){
        io.setOutputPercentage(0);
    }
}
