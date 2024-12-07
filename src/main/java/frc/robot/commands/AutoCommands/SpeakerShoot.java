package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;

public class SpeakerShoot extends Command{
    private final Shooter shooter;
    private final Rollers rollers;
    private final Pivot m_ShooterPivot;
    private boolean ending;
    private double desiredAngle;

    public SpeakerShoot(Shooter shooter, Pivot pivot , Rollers rollers, double desiredAngle){
        this.shooter = shooter;
        this.m_ShooterPivot = pivot;
        this.rollers = rollers;
        this.desiredAngle = desiredAngle;
        ending = false; 
    }

    private enum State {
        START,
        EXTEND,
        SHOOT,
        END
    }

    private State state = State.START;
    private double startTime;

    @Override
    public void initialize(){
        ending = false;
        state = State.START;
    }

    @Override
    public void execute(){
        double timeElapsed = Timer.getFPGATimestamp() - startTime;

        shooter.state = ShooterState.SPEAKER_ACCELERATING;

        if (desiredAngle == 0){
        m_ShooterPivot.setDesiredAngle(GlobalVariables.getInstance().speakerToAngle());
        } else {
            m_ShooterPivot.setDesiredAngle(Constants.VisionConstants.y_ArmAngle[0]);
        }
        switch (state) {
            case START:
                if (shooter.state == ShooterState.READY) {
                    startTime = Timer.getFPGATimestamp();
                    state = State.EXTEND;
                }
                break;
            case EXTEND:
                if (timeElapsed < 0.1) {
                    rollers.state = RollerState.PREPARE_FEED;
                } else {
                    state = State.SHOOT;
                }
                break;
            case SHOOT:
                if (timeElapsed < 1.2) {
                    rollers.state = RollerState.FEEDING;
                } else {
                    state = State.END;
                }
                break;
            case END:
                ending = true;
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted){
        state = State.START;
        shooter.state = ShooterState.IDLE;
        rollers.state = RollerState.IDLE;
    }

    @Override
    public boolean isFinished(){
        return ending;
    }

}
