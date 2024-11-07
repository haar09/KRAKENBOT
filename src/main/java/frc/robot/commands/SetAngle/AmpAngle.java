package frc.robot.commands.SetAngle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.pivot.Pivot;

public class AmpAngle extends Command{
    private final Pivot m_ShooterPivot;

    public AmpAngle(Pivot shooter) {
        this.m_ShooterPivot = shooter;
        addRequirements(m_ShooterPivot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_ShooterPivot.setDesiredAngle(Constants.VisionConstants.kAmpAngle);
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