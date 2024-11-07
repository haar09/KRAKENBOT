package frc.robot.commands.SetAngle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.pivot.Pivot;

public class ShooterAutoAngle extends Command{
    private final Pivot m_ShooterPivot;

    public ShooterAutoAngle(Pivot shooter) {
        this.m_ShooterPivot = shooter;

        addRequirements(m_ShooterPivot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {                
        m_ShooterPivot.setDesiredAngle(GlobalVariables.getInstance().speakerToAngle());
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