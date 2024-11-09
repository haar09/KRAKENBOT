package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.GlobalVariables;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LEDSubsystem;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase{
    
    private final ShooterIO io;
    private final LEDSubsystem ledSubsystem;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public static Shooter create(LEDSubsystem ledSubsystem) {
        return new Shooter(Robot.isReal() ? new RealShooter() : new NoShooter(), ledSubsystem);    
    }

    public Shooter(ShooterIO io, LEDSubsystem ledSubsystem){
        this.io = io;
        this.ledSubsystem = ledSubsystem;
        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                null,          // Use default timeout (10 s)
                                    // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("rightState", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts)-> io.setSysIdVoltage(volts), // hangi motor oldugunu RealShooterdan sec
                null,
                this));
    }

    public enum ShooterState {
        IDLE,
        PRESPEED,
        SPEAKER_ACCELERATING,
        AMP_ACCELERATING,
        READY,
    }

    public ShooterState state = ShooterState.IDLE;

    @Override
    public void periodic() {
        switch (state) {
            case IDLE:
                SmartDashboard.putBoolean("shooterReady", false);
                ledSubsystem.isAccelerating = false;
                ledSubsystem.isReady = false;
                io.neutralMotors();
                if (GlobalVariables.getInstance().speakerDistance <= 4){
                    state = ShooterState.PRESPEED;
                }
                break;
            case PRESPEED:
                if (GlobalVariables.getInstance().speakerDistance > 4){
                    state = ShooterState.IDLE;
                }
                io.setVelocity(ShooterConstants.kAmpSpeedLeft, ShooterConstants.kAmpSpeedRight);
                break;
            case SPEAKER_ACCELERATING:
                ledSubsystem.isAccelerating = true;
                io.setVelocity(ShooterConstants.kSpeakerSpeedLeft, ShooterConstants.kSpeakerSpeedRight);
                if (inputs.leftMotionMagicError < 3 && inputs.rightMotionMagicError < 3) {
                    SmartDashboard.putBoolean("shooterReady", true);
                    state = ShooterState.READY;
                    ledSubsystem.isReady = true;
                }
                break;
            case AMP_ACCELERATING:
                ledSubsystem.isAccelerating = true;
                io.setVelocity(ShooterConstants.kAmpSpeedLeft, ShooterConstants.kAmpSpeedRight);
                if (inputs.leftMotionMagicError < 3 && inputs.rightMotionMagicError < 3) {
                    SmartDashboard.putBoolean("shooterReady", true);
                    state = ShooterState.READY;
                    ledSubsystem.isReady = true;
                }
                break;
            case READY:
                ledSubsystem.isAccelerating = false;
                break;
        }

        SmartDashboard.putNumber("leftSpeed", inputs.leftVelocityRps);
        SmartDashboard.putNumber("rightSpeed", inputs.rightVelocityRps);

        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        SmartDashboard.putString("Shooter State", state.toString());
        Logger.recordOutput("Shooter/State", state.toString());
    }

    private final SysIdRoutine m_sysIdRoutine;
        
    public Command sysIdQuasistatic(Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public Command setStateCommand(ShooterState state) {
        return startEnd(() -> this.state = state, () -> this.state = ShooterState.IDLE).withName("Shooter "+state.toString());
    }

}
