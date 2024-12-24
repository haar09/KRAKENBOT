package frc.robot.subsystems.ampMechanism;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class AmpMechanism extends SubsystemBase {
    private final AmpMechanismIO io;
    private final AmpMechanismIOInputsAutoLogged inputs = new AmpMechanismIOInputsAutoLogged();
    private final Alert disconnectedAlert;

    public static AmpMechanism create() {
        return new AmpMechanism(Robot.isReal() ? new RealAmpMechanism() : new NoAmpMechanism());
    }

    public AmpMechanism(AmpMechanismIO io) {
        this.io = io;
        this.disconnectedAlert = new Alert("Amp Mechanism is disconnected.", AlertType.kWarning);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Amp Mechanism", inputs);
        disconnectedAlert.set(!inputs.motorConnected);
    }

    public void setOutputPercentage(double percentage) {
        io.setOutputPercentage(percentage);
    }

    public void stop(){
        io.setOutputPercentage(0);
    }
}
