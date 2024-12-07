package frc.robot.commands.SetAngle;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.pivot.Pivot;

public class ShooterSetAngle extends Command{
    private final Pivot m_ShooterPivot;
    private final LoggedNetworkNumber customAngle = new LoggedNetworkNumber("/SmartDashboard/customAngle", VisionConstants.y_ArmAngle[0]);

    public ShooterSetAngle(Pivot shooter) {
        this.m_ShooterPivot = shooter;
        addRequirements(m_ShooterPivot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_ShooterPivot.setDesiredAngle(customAngle.get());
    }

    @Override
    public void end(boolean interrupted) {
        m_ShooterPivot.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}