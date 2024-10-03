package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;

public class DriveToNote extends Command{
        
    private final XboxController driverJoystick;
    private final Intake intake;
    private final Extender extender;
    private final LEDSubsystem ledSubsystem;

    public DriveToNote(XboxController driverController,
    Intake intake, Extender extender, LEDSubsystem ledSubsystem){
        this.driverJoystick = driverController;
        this.intake = intake;
        this.extender = extender;
        this.ledSubsystem = ledSubsystem;
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        ledSubsystem.isAutodrive = true;
        
        if (!GlobalVariables.getInstance().extenderFull) {
            intake.setOutputPercentage(IntakextenderConstants.kIntakeMotorSpeed);
            extender.setOutputPercentage(IntakextenderConstants.kExtenderSpeed);
        } else {
            driverJoystick.setRumble(RumbleType.kBothRumble, 0.6);
        }
    }

    public void end(boolean interrupted){
        ledSubsystem.isAutodrive = false;
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}