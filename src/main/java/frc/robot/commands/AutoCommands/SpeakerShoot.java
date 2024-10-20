package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.ShooterPivot;

public class SpeakerShoot extends Command{
    private final Shooter shooter;
    private final Extender extender;
    private final Intake intake;
    private final ShooterPivot m_ShooterPivot;
    private boolean ending;
    private double desiredAngle;

    public SpeakerShoot(Shooter shooter, ShooterPivot pivot ,Extender extender, Intake intake, double desiredAngle){
        this.shooter = shooter;
        this.m_ShooterPivot = pivot;
        this.extender = extender;
        this.intake = intake;
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
        shooter.state = ShooterState.IDLE;
    }

    @Override
    public void execute(){
        double timeElapsed = Timer.getFPGATimestamp() - startTime;

        shooter.setSpeakerSpeed();

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
                    extender.setOutputPercentage(-IntakextenderConstants.kExtenderBackSpeed);
                } else {
                    state = State.SHOOT;
                }
                break;
            case SHOOT:
                timeElapsed = Timer.getFPGATimestamp() - startTime;
                if (timeElapsed < 1) {
                    shooter.setSpeakerSpeed();
                    extender.setOutputPercentage(1);
                    intake.setOutputPercentage(0.6);
                } else {
                    state = State.END;
                }
                break;
            case END:
                shooter.stopShooter();
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
