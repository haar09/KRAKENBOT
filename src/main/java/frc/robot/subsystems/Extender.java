package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.util.NEO;

public class Extender extends SubsystemBase {
    private final NEO extenderMotor;

    public Extender() {
        extenderMotor = new NEO(IntakextenderConstants.kExtenderMotorId, IntakextenderConstants.kExtenderMotorReversed, IdleMode.kBrake);
    }

    public void setOutputPercentage(double percentage) {
        extenderMotor.set(percentage);
    }

    public void stop(){
        extenderMotor.set(0);
    }
}
