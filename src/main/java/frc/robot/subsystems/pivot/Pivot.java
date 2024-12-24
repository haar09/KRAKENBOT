package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Pivot extends SubsystemBase{
    private final PivotIO pivot;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private Pose3d pivotPose = new Pose3d(0, 0, 0, new Rotation3d(0,0,0));
    private final Alert disconnectedAlert;

    private final PivotVisualizer positionVisualizer = new PivotVisualizer(new Color8Bit(255, 0, 0));
    private final PivotVisualizer setPointVisualizer = new PivotVisualizer(new Color8Bit(0, 0, 255));

    public static Pivot create() {
        return new Pivot(Robot.isReal() ? new RealPivot() : new NoPivot());
    }

    public Pivot(PivotIO pivot) {
        this.pivot = pivot;
        setPointVisualizer.setState(-180);
        this.disconnectedAlert = new Alert("Pivot is disconnected.", AlertType.kWarning);
    }

    @Override
    public void periodic() {
        pivot.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        disconnectedAlert.set(!inputs.motorConnected);

        pivotPose = new Pose3d(
            pivot.getAngle()*0.1,
            0,
            -pivot.getAngle()*0.005
            , new Rotation3d(0, -pivot.getAngle(), 0));
        Logger.recordOutput("Pivot/Angle", pivotPose);
        
        positionVisualizer.setState(getAngle());
        Logger.recordOutput("Pivot/Position", positionVisualizer.mech);
        Logger.recordOutput("Pivot/SetPoint", setPointVisualizer.mech);

        
        setPointVisualizer.setState(0);

        SmartDashboard.putNumber("Shooter Angle", Math.toDegrees(pivot.getAngle()));
    }

    public void setDesiredAngle(double angle) {
        pivot.setDesiredAngle(angle);
        setPointVisualizer.setState(Math.toRadians(angle));
    }

    public void stop() {
        pivot.stop();
    }

    public double getAngle() {
        return pivot.getAngle();
    }
}