package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.VisionConstants;

public class GlobalVariables {
    private static GlobalVariables instance = new GlobalVariables();
    public double customRotateSpeed;
    public boolean extenderFull;
    public double speakerDistance;
    public DriverStation.Alliance alliance;
    public boolean[] detectedList = {false, false, false};

    private GlobalVariables() {
        customRotateSpeed = 0;
        extenderFull = false;
        speakerDistance = 0;
        alliance = null;
        thetaController.enableContinuousInput(0, 360);
        thetaController.setTolerance(1.5);
        thetaController2.enableContinuousInput(0, 360);
    }

    public void isDetected(boolean detected, int index) {
        detectedList[index] = detected;
        if (detectedList[0] || detectedList[1] || detectedList[2]) {
            SmartDashboard.putBoolean("Limelight Target", true);
        } else {
            SmartDashboard.putBoolean("Limelight Target", false);
        }
    }

    public double speakerToAngle() {
        if (speakerDistance < 1) {
            return (VisionConstants.y_ArmAngle[0]);   
        } else if (speakerDistance < 3.5) {
            return VisionConstants.angleFunction.value(speakerDistance);
        }
        else {
            return 0;
        }
    }

    private Translation2d kSpeakerApriltagPose;
    private PIDController thetaController = new PIDController(PIDConstants.kPLimeLightRotate, 0, PIDConstants.kDLimeLightRotate);
    
    public double getSpeakerAngle(Pose2d swervePose) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            kSpeakerApriltagPose = VisionConstants.kTagLayout.getTagPose(4).get().getTranslation().toTranslation2d().minus(new Translation2d(0.1, 0));            
        } else {
            kSpeakerApriltagPose = VisionConstants.kTagLayout.getTagPose(7).get().getTranslation().toTranslation2d().plus(new Translation2d(0.1, 0));        
        }
        return thetaController.calculate(swervePose.getRotation().getDegrees() ,kSpeakerApriltagPose.minus(swervePose.getTranslation()).getAngle().getDegrees());
    }

    private PIDController thetaController2 = new PIDController(PIDConstants.kPObjectRotate, 0, PIDConstants.kDObjectRotate);

    public double getObjectAngle(double angle){
        return thetaController2.calculate(angle);
    }

    double feed_rot;
    public double getFeedAngle(Pose2d swervePose) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            feed_rot = 33.3;
        } else {
            feed_rot = 146.7;
        }
        return thetaController.calculate(swervePose.getRotation().getDegrees() , feed_rot);
    }

    public static GlobalVariables getInstance() {
        return instance;
    }
}