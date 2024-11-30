package frc.robot.subsystems.rollers.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimIntake implements IntakeIO{
    private final DCMotorSim sim;
    private double appliedVoltage = 0;

    public SimIntake() {
        sim = new DCMotorSim(DCMotor.getNEO(1), 2/1, 0.001);
    }

    @Override
    public void setOutputPercentage(double percentage) {
        appliedVoltage = percentage * RobotController.getBatteryVoltage();
        sim.setInputVoltage(appliedVoltage);
    }

    @Override
    public void stop(){
        sim.setInputVoltage(0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            sim.setInputVoltage(0);
        }

        sim.update(0.02);
        inputs.velocityRotationsPerMinute = sim.getAngularVelocityRPM();
        inputs.appliedVolts = appliedVoltage;
        inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    }
}
