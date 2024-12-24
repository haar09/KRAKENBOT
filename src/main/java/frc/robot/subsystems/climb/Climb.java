package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    private final Alert disconnectedAlert;

    public static Climb create() {
        return new Climb(Robot.isReal() ? new RealClimb() : new NoClimb());
    }

    public Climb(ClimbIO io) {
        this.io = io;
        this.disconnectedAlert = new Alert("Climb is disconnected.", AlertType.kWarning);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
        disconnectedAlert.set(!inputs.motorConnected);
    }

    public void setOutputPercentage(double percentage) {
        io.setOutputPercentage(percentage);
    }

    public void stop(){
        io.setOutputPercentage(0);
    }
}
