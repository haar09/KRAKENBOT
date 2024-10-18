package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbCmd extends Command{
    private final Climb climb;
    private final Supplier<Boolean> take, out;
    private boolean endv;

    public ClimbCmd(Supplier<Boolean> take, Supplier<Boolean> out,
    Climb climb){
        this.take = take;
        this.out = out;
        this.climb = climb;
        this.endv = false;
        addRequirements(climb);
    }

    @Override
    public void initialize(){
        endv = false;
    }

    @Override
    public void execute(){
        // get joystick
        boolean takeSpeed = take.get();
        boolean outSpeed = out.get();

        if (takeSpeed) {
            climb.setOutputPercentage(0.73);
        } else if (outSpeed) {
                climb.setOutputPercentage(-0.73);
        } else {
            climb.setOutputPercentage(0);
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return endv;
    }
}