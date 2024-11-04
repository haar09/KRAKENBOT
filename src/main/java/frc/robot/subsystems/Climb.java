package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.NEO;

public class Climb extends SubsystemBase {
    private final NEO climbMotor;

    public Climb() {
        climbMotor = new NEO(ClimbConstants.kClimbMotorId, ClimbConstants.kClimbMotorReversed, IdleMode.kBrake);
    }   

    public void setOutputPercentage(double percentage) {
        climbMotor.set(percentage);
    }

    public void stop(){
        climbMotor.set(0);
    }
}
