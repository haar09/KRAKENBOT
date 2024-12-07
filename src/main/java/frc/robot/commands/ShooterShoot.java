package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;

public class ShooterShoot extends Command{
    private final Supplier<Double> rightTrigger;
    private final Shooter shooter;
    private final Rollers rollers;
    private boolean ending, amp;
    private final XboxController operatorController;

    public ShooterShoot(Supplier<Double> rightTrigger ,Shooter shooter, Rollers rollers, Boolean amp, XboxController operatorController){
        this.shooter = shooter;
        this.rollers = rollers;
        this.rightTrigger = rightTrigger;
        this.amp = amp;
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
        if (amp) {
            shooter.state = ShooterState.AMP_ACCELERATING;
        } else {
            shooter.state = ShooterState.SPEAKER_ACCELERATING;
        }
    }

    @Override
    public void execute() {
        double timeElapsed = Timer.getFPGATimestamp() - startTime;

        switch (state) {
            case START:
                if (shooter.state == ShooterState.READY) {
                    if (rightTrigger.get() > 0.3) {
                        operatorController.setRumble(RumbleType.kBothRumble, 0);
                        startTime = Timer.getFPGATimestamp();
                        state = State.EXTEND;
                    } else {
                        operatorController.setRumble(RumbleType.kBothRumble, 0.5);
                    }
                } else {
                    if (rightTrigger.get() > 0.3) {
                        operatorController.setRumble(RumbleType.kRightRumble, 1);
                    } else { 
                        operatorController.setRumble(RumbleType.kBothRumble, 0);
                    }
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
    }

    @Override
    public boolean isFinished(){
        return ending;
    }

}