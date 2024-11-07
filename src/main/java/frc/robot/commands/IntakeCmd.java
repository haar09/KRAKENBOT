package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.extender.Extender;

public class IntakeCmd extends Command{
    private final Intake intake;
    private final Extender extender;
    private final Supplier<Boolean> take, out, hard;
    private final XboxController driverController;
    private boolean isIntaking;
    private double intakeTimeout;
    private final BeamBreak beamBreak;

    public IntakeCmd(Supplier<Boolean> take, Supplier<Boolean> out, Supplier<Boolean> hard,
    Intake intake, Extender extender, XboxController driverController, BeamBreak beamBreak){
        this.take = take;
        this.out = out;
        this.hard = hard;
        this.intake = intake;
        this.extender = extender;
        this.driverController = driverController;
        this.beamBreak = beamBreak;
        isIntaking = false;
        intakeTimeout = 0;
    }

    @Override
    public void initialize(){
        intakeTimeout = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        // get joystick
        boolean takeSpeed = take.get();
        boolean outSpeed = out.get();

        if (takeSpeed) {
            if (hard.get()) {
                intake.setOutputPercentage(0.8);
                extender.setOutputPercentage(0.8);
            } else {
                if (!beamBreak.upper_value) {
                    intake.setOutputPercentage(IntakextenderConstants.kIntakeMotorSpeed);
                    extender.setOutputPercentage(IntakextenderConstants.kExtenderSpeed);
                } else {
                    driverController.setRumble(RumbleType.kBothRumble, 0.6);
                }
            }
        } else if (outSpeed) {
            isIntaking = false;
            if (hard.get()) {
                intake.setOutputPercentage(-0.8);
                extender.setOutputPercentage(-0.8);
            } else {
                intake.setOutputPercentage(-IntakextenderConstants.kIntakeMotorSpeed);
                extender.setOutputPercentage(-IntakextenderConstants.kExtenderSpeed);
            }

        } else if (beamBreak.lower_value && !beamBreak.upper_value) {
            isIntaking = true;
            intake.setOutputPercentage(IntakextenderConstants.kIntakeMotorSpeed);
            extender.setOutputPercentage(IntakextenderConstants.kExtenderSpeed);

        } else if (isIntaking){
            if (beamBreak.upper_value) {
                isIntaking = false;
            } else {
                intake.setOutputPercentage(IntakextenderConstants.kIntakeMotorSpeed);
                extender.setOutputPercentage(IntakextenderConstants.kExtenderSpeed);
            }
            if (Timer.getFPGATimestamp()-intakeTimeout > 5){
                isIntaking = false;
            }
        } else {
            driverController.setRumble(RumbleType.kBothRumble, 0);
            intake.setOutputPercentage(0);
            extender.setOutputPercentage(0);

        }

        if (beamBreak.upper_value && !outSpeed && !hard.get()) {
            intake.setOutputPercentage(0);
            extender.setOutputPercentage(0);
        }
        
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}