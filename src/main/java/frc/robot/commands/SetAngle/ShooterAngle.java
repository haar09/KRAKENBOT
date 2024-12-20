package frc.robot.commands.SetAngle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.pivot.Pivot;

public class ShooterAngle extends Command{
    private final Pivot m_ShooterPivot;

    public ShooterAngle(Pivot shooter) {
        this.m_ShooterPivot = shooter;
        addRequirements(m_ShooterPivot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_ShooterPivot.setDesiredAngle(VisionConstants.y_ArmAngle[0]); //subwoofer dibi
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