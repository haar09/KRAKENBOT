package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;

public class FeedAuto extends Command{
    private final Shooter shooter;
    private final Rollers rollers;
    private final Pivot shooterPivot;
    private boolean ending;
    private final XboxController operatorController;

    public FeedAuto(Shooter shooter, Rollers rollers, Pivot shooterPivot, XboxController operatorController){
        this.shooter = shooter;
        this.rollers = rollers;
        this.shooterPivot = shooterPivot;
        this.operatorController = operatorController;
        ending = false;
        addRequirements(shooter); 
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
    public void execute() {
        double timeElapsed = Timer.getFPGATimestamp() - startTime;

        if (GlobalVariables.getInstance().extenderFull) {
            shooter.state = ShooterState.SPEAKER_ACCELERATING;
        } else {
            operatorController.setRumble(RumbleType.kBothRumble, 1);
            if (state == State.START) {
                return;
            }
        }

        shooterPivot.setDesiredAngle(Constants.VisionConstants.kFeedAngle);

        switch (state) {
            case START:
                if (shooter.state == ShooterState.READY) {
                    operatorController.setRumble(RumbleType.kBothRumble, 0);
                    startTime = Timer.getFPGATimestamp();
                    state = State.EXTEND;
                    operatorController.setRumble(RumbleType.kBothRumble, 1);
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
                if (timeElapsed < 2) {
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
        SmartDashboard.putBoolean("shooterReady", false);
    }

    @Override
    public boolean isFinished(){
        return ending;
    }

}
