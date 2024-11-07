package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ObjectDetection;
import java.util.ArrayList;

public class DriveToNote extends Command{
        
    private final XboxController driverJoystick;
    private final LEDSubsystem ledSubsystem;
    private final BeamBreak beamBreak;
    private final CommandSwerveDrivetrain drivetrain;
    private final ObjectDetection objectDetection;
    private ArrayList<Boolean> m_targetList;
    private final int MAX_ENTRIES = 8;
    private double startTime;


    private final SwerveRequest.RobotCentric robotOriented = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want r-centric driving in open loop

    public DriveToNote(
    CommandSwerveDrivetrain drivetrain, ObjectDetection objectDetection, XboxController driverController, LEDSubsystem ledSubsystem, BeamBreak beamBreak){
        this.drivetrain = drivetrain;
        this.objectDetection = objectDetection;
        this.driverJoystick = driverController;
        this.ledSubsystem = ledSubsystem;
        this.beamBreak = beamBreak;
        thetaController.enableContinuousInput(0, 360);
        m_targetList = new ArrayList<Boolean>(MAX_ENTRIES);
    }
    
    private PIDController thetaController = new PIDController(PIDConstants.kPObjectRotate, 0, PIDConstants.kDObjectRotate);

    @Override
    public void initialize(){
        thetaController.reset();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        m_targetList.add(objectDetection.targetValid());
        driverJoystick.setRumble(RumbleType.kRightRumble, 0.3);  

        if(thetaController.getPositionError() > 3) {
        drivetrain.setControl(robotOriented.withVelocityX(0).withRotationalRate(thetaController.calculate(objectDetection.getTX())));
        } else {
            drivetrain.setControl(robotOriented.withVelocityX(-2).withVelocityY(0).withRotationalRate(0));
        }

        ledSubsystem.isAutodrive = true;

        if (m_targetList.size() >= MAX_ENTRIES) {
            m_targetList.remove(0);
        }
    }

    public boolean isDetected() {
        double positive = 0;
        for (Boolean target : m_targetList) {
          if (target) {
            positive += 1;
          }
        }

        return positive > 1;
    }

    @Override
    public void end(boolean interrupted){
        ledSubsystem.isAutodrive = false;
        driverJoystick.setRumble(RumbleType.kRightRumble, 0);
    }

    @Override
    public boolean isFinished(){
    return !isDetected() || beamBreak.lower_value || Timer.getFPGATimestamp() - startTime > 5
        || (driverJoystick.getBButton() && Timer.getFPGATimestamp() - startTime > 0.3)
        || (driverJoystick.getRightStickButton() && Timer.getFPGATimestamp() - startTime > 0.3);
    }
}