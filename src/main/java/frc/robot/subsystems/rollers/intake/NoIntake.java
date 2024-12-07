package frc.robot.subsystems.rollers.intake;

import edu.wpi.first.wpilibj.RobotController;

public class NoIntake implements IntakeIO{
    private double appliedVoltage = 0;

    public NoIntake() {
    }

    @Override
    public void setOutputPercentage(double percentage) {
        appliedVoltage = percentage * RobotController.getBatteryVoltage();
    }

    @Override
    public void stop(){
        appliedVoltage = 0;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = appliedVoltage;
    }
}
