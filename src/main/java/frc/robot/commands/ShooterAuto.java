package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;

public class ShooterAuto extends Command{
    private final Shooter shooter;
    private final Rollers rollers;
    private final CommandSwerveDrivetrain drivetrain;
    private final Pivot shooterPivot;
    private boolean ending;
    private final XboxController driverController;

    public ShooterAuto(Shooter shooter, Rollers rollers,
    CommandSwerveDrivetrain drivetrain, Pivot shooterPivot, XboxController driverController){
        this.shooter = shooter;
        this.rollers = rollers;
        this.drivetrain = drivetrain;
        this.shooterPivot = shooterPivot;
        this.driverController = driverController;
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

        if (!GlobalVariables.getInstance().extenderFull) {
            driverController.setRumble(RumbleType.kBothRumble, 1);
        }

        shooterPivot.setDesiredAngle(GlobalVariables.getInstance().speakerToAngle());

        Logger.recordOutput("Auto Shoot/State", state.toString());
        Logger.recordOutput("Auto Shoot/vx", Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond));
        Logger.recordOutput("Auto Shoot/vy", Math.abs(drivetrain.getState().Speeds.vyMetersPerSecond));       
        Logger.recordOutput("Auto Shoot/vw", drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()); 

        switch (state) {
            case START:
                if (shooter.state == ShooterState.READY) {
                        if (GlobalVariables.getInstance().speakerToAngle() > -1 &&
                        Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond) < 0.1 && Math.abs(drivetrain.getState().Speeds.vyMetersPerSecond) < 0.1
                        && Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) < 7) {
                            Logger.recordOutput("Auto Shoot/Conditions Met", true);
                            driverController.setRumble(RumbleType.kBothRumble, 0);
                            startTime = Timer.getFPGATimestamp();
                            state = State.EXTEND;
                        }
                } else if (GlobalVariables.getInstance().speakerDistance < 3.8) {
                    shooter.state = ShooterState.SPEAKER_ACCELERATING;
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
        driverController.setRumble(RumbleType.kBothRumble, 0);
        Logger.recordOutput("Auto Shoot/Conditions Met", false);
        state = State.START;
        shooter.state = ShooterState.IDLE;
        rollers.state = RollerState.IDLE;
    }

    @Override
    public boolean isFinished(){
        return ending;
    }

}
