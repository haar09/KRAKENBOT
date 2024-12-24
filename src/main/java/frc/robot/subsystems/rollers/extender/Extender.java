package frc.robot.subsystems.rollers.extender;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;

public class Extender {
    private final ExtenderIO io;
    private final ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();
    private final Alert disconnectedAlert;

    public static Extender create() {
        return new Extender(Robot.isReal() ? new RealExtender() : new NoExtender());
    }

    public Extender(ExtenderIO io) {
        this.io = io;
        this.disconnectedAlert = new Alert("Extender is disconnected.", AlertType.kWarning);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Extender", inputs);
        disconnectedAlert.set(!inputs.motorConnected);
    }

    public void setOutputPercentage(double percentage) {
        io.setOutputPercentage(percentage);
    }

    public void stop(){
        io.setOutputPercentage(0);
    }
}
