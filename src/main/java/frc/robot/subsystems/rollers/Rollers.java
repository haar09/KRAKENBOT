package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.rollers.extender.Extender;
import frc.robot.subsystems.rollers.intake.Intake;

public class Rollers extends SubsystemBase{
    private final Extender extender;
    private final Intake intake;
    private final BeamBreak beamBreak;
    private final LEDSubsystem ledSubsystem;

    public Rollers(Extender extender, Intake intake, BeamBreak beamBreak, LEDSubsystem ledSubsystem) {
        this.extender = extender;
        this.intake = intake;
        this.beamBreak = beamBreak;
        this.ledSubsystem = ledSubsystem;
    }

    public enum RollerState{
        IDLE,
        FLOOR_INTAKING,
        INDEXING,
        STUCK_INTAKING,
        EJECTING,
        STUCK_EJECTING,
        PREPARE_FEED,
        FEEDING
    }
    
    public RollerState state = RollerState.IDLE;
    
    @Override
    public void periodic() {
        extender.periodic();
        intake.periodic();

        switch (state) {
            case IDLE -> {
                extender.setOutputPercentage(0);
                intake.setOutputPercentage(0);
            }
            case FLOOR_INTAKING -> {
                if (beamBreak.upper_value) {
                    state = RollerState.IDLE;
                }
                if (beamBreak.lower_value) {
                    state = RollerState.INDEXING;
                }
                extender.setOutputPercentage(IntakextenderConstants.kIntakeMotorSpeed);
                intake.setOutputPercentage(IntakextenderConstants.kExtenderSpeed);
            }
            case INDEXING -> {
                if (beamBreak.upper_value) {
                    state = RollerState.IDLE;
                }
                extender.setOutputPercentage(IntakextenderConstants.kIntakeMotorSpeed);
                intake.setOutputPercentage(IntakextenderConstants.kExtenderSpeed);
            }
            case STUCK_INTAKING -> {
                extender.setOutputPercentage(0.8);
                intake.setOutputPercentage(0.8);
            }
            case EJECTING -> {
                extender.setOutputPercentage(-IntakextenderConstants.kIntakeMotorSpeed);
                intake.setOutputPercentage(-IntakextenderConstants.kExtenderSpeed);
            }
            case STUCK_EJECTING -> {
                extender.setOutputPercentage(-0.8);
                intake.setOutputPercentage(-0.8);
            }
            case PREPARE_FEED -> {
                extender.setOutputPercentage(-IntakextenderConstants.kExtenderBackSpeed);
            }
            case FEEDING -> {
                extender.setOutputPercentage(1);
                intake.setOutputPercentage(0.6);
            }
        }

        if (beamBreak.upper_value) {
            ledSubsystem.isNote = true;
            if (GlobalVariables.getInstance().speakerToAngle() >= 0) {
                ledSubsystem.isInRange = true;
            } else {
                ledSubsystem.isInRange = false;
            }
            ledSubsystem.isIntaking = false;
        } else {
            ledSubsystem.isIntaking = state == RollerState.INDEXING;
            ledSubsystem.isNote = false;
            ledSubsystem.isInRange = false;
        }

        Logger.recordOutput("Rollers/State", state.toString());
        SmartDashboard.putString("Roller State", state.toString());
    }

    public Command setStateCommand(RollerState state) {
        return startEnd(() -> this.state = state, () -> this.state = RollerState.IDLE).withName("Rollers "+state.toString());
    }
}
