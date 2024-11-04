package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.util.NEO;

public class Intake extends SubsystemBase {
    private final NEO intakeMotor;
    private final DigitalInput lower_beambreak;
    public boolean intakeStart;

    public Intake() {
        intakeMotor = new NEO(IntakextenderConstants.kIntakeMotorId, IntakextenderConstants.kIntakeMotorReversed, IdleMode.kCoast);

        lower_beambreak = new DigitalInput(2);

        intakeStart = false;
    }   

    @Override
    public void periodic() {
        intakeStart = !lower_beambreak.get();
    }
        

    public void setOutputPercentage(double percentage) {
        intakeMotor.set(percentage);
    }

    public void stop(){
        intakeMotor.set(0);
    }
}
