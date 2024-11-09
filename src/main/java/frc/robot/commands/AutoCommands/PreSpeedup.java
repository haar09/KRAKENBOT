package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class PreSpeedup extends Command{
    private Shooter shooter;

    public PreSpeedup(Shooter shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        shooter.state = ShooterState.SPEAKER_ACCELERATING;
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}