package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.extender.Extender;
import frc.robot.subsystems.pivot.Pivot;

public class FeedAuto extends Command{
    private final Shooter shooter;
    private final Extender extender;
    private final Intake intake;
    private final Pivot shooterPivot;
    private boolean ending;
    private final XboxController operatorController;

    public FeedAuto(Shooter shooter, Intake intake,Extender extender, Pivot shooterPivot, XboxController operatorController){
        this.shooter = shooter;
        this.extender = extender;
        this.intake = intake;
        this.shooterPivot = shooterPivot;
        this.operatorController = operatorController;
        ending = false;
        addRequirements(shooter, extender); 
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
        shooter.state = ShooterState.IDLE;
    }

    @Override
    public void execute() {
        double timeElapsed = Timer.getFPGATimestamp() - startTime;

        if (GlobalVariables.getInstance().extenderFull) {
            shooter.setSpeakerSpeed();
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
                    extender.setOutputPercentage(-IntakextenderConstants.kExtenderBackSpeed);
                } else {
                    state = State.SHOOT;
                }
                break;
            case SHOOT:
                if (timeElapsed < 2) {
                    shooter.setSpeakerSpeed();
                    extender.setOutputPercentage(1);
                    intake.setOutputPercentage(0.6);
                } else {
                    state = State.END;
                }
                break;
            case END:
                ending = true;
                end(false);
                extender.setOutputPercentage(0);
                shooter.stopShooter();
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted){
        state = State.START;
        shooter.state = ShooterState.IDLE;
        intake.setOutputPercentage(0);
        extender.setOutputPercentage(0);
        shooter.stopShooter();    
        SmartDashboard.putBoolean("shooterReady", false);

        GlobalVariables.getInstance().customRotateSpeed = 0;
    }

    @Override
    public boolean isFinished(){
        return ending;
    }

}
