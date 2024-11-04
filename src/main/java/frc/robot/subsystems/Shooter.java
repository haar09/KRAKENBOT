package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ShooterConstants;
import frc.robot.GlobalVariables;
import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase{

    private final TalonFX leftMotor = new TalonFX(ShooterConstants.kShooterMotorLeftId);
    private final MotionMagicVelocityVoltage leftMotorVoltageRequest;

    private final TalonFX rightMotor = new TalonFX(ShooterConstants.kShooterMotorRightId);
    private final VoltageOut rightMotorVoltageRequest;

    public final LEDSubsystem ledSubsystem;

    public double time;

    public Shooter(LEDSubsystem m_LedSubsystem) {
        ledSubsystem = m_LedSubsystem;

        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);

        leftMotor.setInverted(ShooterConstants.kShooterMotorLeftReversed);
        rightMotor.setInverted(ShooterConstants.kShooterMotorRightReversed);

        leftMotor.getConfigurator().apply(ShooterConstants.leftMotorConfig);
        leftMotorVoltageRequest = new MotionMagicVelocityVoltage(0);
        rightMotorVoltageRequest = new VoltageOut(0);

        BaseStatusSignal.setUpdateFrequencyForAll(250, 
        rightMotor.getPosition(),
        rightMotor.getVelocity(),
        rightMotor.getMotorVoltage());
    }

    public enum ShooterState {
        IDLE,
        ACCELERATING,
        READY
    }

    public ShooterState state = ShooterState.IDLE;
    private double startTime;

    public void setSpeakerSpeed() {
        switch (state) {
            case IDLE:
                SmartDashboard.putBoolean("shooterReady", false);
                leftMotor.setControl(leftMotorVoltageRequest.withVelocity(64.3));
                rightMotor.setControl(rightMotorVoltageRequest.withOutput((ShooterConstants.kSpeakerSpeedRight * ShooterConstants.kVoltageCompensation)));
                startTime = Timer.getFPGATimestamp();
                state = ShooterState.ACCELERATING; 
                ledSubsystem.isAccelerating = true;
                break;
            case ACCELERATING:
                if (leftMotor.getVelocity().getValueAsDouble() >= 64 && rightMotor.getVelocity().getValueAsDouble() >= 52) {
                    SmartDashboard.putBoolean("shooterReady", true);
                    state = ShooterState.READY;
                    ledSubsystem.isReady = true;
                } else if (Timer.getFPGATimestamp() - startTime > 2) {
                    stopShooter();
                    state = ShooterState.IDLE;
                }
            break;
            case READY:
                leftMotorVoltageRequest.withVelocity(64.3);
                rightMotor.setControl(rightMotorVoltageRequest.withOutput(ShooterConstants.kSpeakerSpeedRight * ShooterConstants.kVoltageCompensation));
                ledSubsystem.isAccelerating = false;
                break;
        }
    }

    public void setAmpSpeed() {
        switch (state) {
            case IDLE:
                SmartDashboard.putBoolean("shooterReady", false);
                leftMotor.setControl(leftMotorVoltageRequest.withVelocity(21.4));
                rightMotor.setControl(rightMotorVoltageRequest.withOutput((ShooterConstants.kAmpSpeedRight * ShooterConstants.kVoltageCompensation)));
                startTime = Timer.getFPGATimestamp();
                state = ShooterState.ACCELERATING; 
                ledSubsystem.isAccelerating = true;
                break;
            case ACCELERATING:
                if (leftMotor.getVelocity().getValueAsDouble() >= 21.2 && rightMotor.getVelocity().getValueAsDouble() >= 21.2) {
                    SmartDashboard.putBoolean("shooterReady", true);
                    state = ShooterState.READY;
                    ledSubsystem.isReady = true;
                } else if (Timer.getFPGATimestamp() - startTime > 2) {
                    stopShooter();
                    state = ShooterState.IDLE;
                }
            break;
            case READY:
                leftMotor.setControl(leftMotorVoltageRequest.withVelocity(21.4));
                rightMotor.setControl(rightMotorVoltageRequest.withOutput((ShooterConstants.kAmpSpeedRight * ShooterConstants.kVoltageCompensation)));
                ledSubsystem.isAccelerating = false;
                break;
        }
    }

    public void preSpeed() {
        leftMotor.setControl(leftMotorVoltageRequest.withVelocity(21.4));
        rightMotor.setControl(rightMotorVoltageRequest.withOutput((ShooterConstants.kAmpSpeedRight * ShooterConstants.kVoltageCompensation)));
    }

    final NeutralOut stopRequest = new NeutralOut();

    public void stopShooter() {
        leftMotor.setControl(stopRequest);
        rightMotor.setControl(stopRequest);
        ledSubsystem.isReady = false;
        ledSubsystem.isAccelerating = false;
    }

    private final VoltageOut m_sysIdControl = new VoltageOut(0);
    
    private final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                null,          // Use default timeout (10 s)
                                       // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("rightState", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts)-> rightMotor.setControl(m_sysIdControl.withOutput(volts.in(Volts))),
                null,
                this));
    
    public Command sysIdQuasistatic(Direction direction) {
            return m_sysIdRoutine.quasistatic(direction);
        }
    public Command sysIdDynamic(Direction direction) {
            return m_sysIdRoutine.dynamic(direction);
        }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("leftSpeed", leftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("rightSpeed", rightMotor.getVelocity().getValueAsDouble());
        
        if (GlobalVariables.getInstance().extenderFull) {
            if (GlobalVariables.getInstance().speakerToAngle() > 0) {
                ledSubsystem.isInRange = true;
            } else {
                ledSubsystem.isInRange = false;
                ledSubsystem.isNote = true;
            }
        } else {
            ledSubsystem.isNote = false;
            ledSubsystem.isInRange = false;
        }
    }

}
