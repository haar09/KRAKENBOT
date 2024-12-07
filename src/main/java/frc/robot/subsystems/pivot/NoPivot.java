package frc.robot.subsystems.pivot;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class NoPivot implements PivotIO {

    private double angle;
    private final SlewRateLimiter anglefall;

    public NoPivot() {
        angle = 0;
        anglefall = new SlewRateLimiter(1);
    }

    @Override
    public void setDesiredAngle(double anglea){
        angle = Math.toRadians(anglea);
    }

    @Override
    public double getAngle(){
        return angle;
    }

    @Override
    public void stop(){
        angle = anglefall.calculate(0);
    }

}