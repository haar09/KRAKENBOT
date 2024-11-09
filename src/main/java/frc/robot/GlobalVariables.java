package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.VisionConstants;
import java.util.HashMap;

public class GlobalVariables {
    private static GlobalVariables instance = new GlobalVariables();
    
    public boolean extenderFull;
    public double speakerDistance;

    public DriverStation.Alliance alliance;
    public HashMap<String, Boolean> detectedMap;

    private GlobalVariables() {
        detectedMap = new HashMap<>();
        extenderFull = false;
        speakerDistance = 0;
        alliance = null;
        thetaController.enableContinuousInput(0, 360);
        thetaController.setTolerance(1.1);
    }

    public void isDetected(boolean detected, String cameraName) {
        detectedMap.put(cameraName, detected);
        SmartDashboard.putBoolean("Limelight Target", detectedMap.values().stream().anyMatch(Boolean::booleanValue));
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
            kSpeakerApriltagPose = VisionConstants.RedSpeakerPose;            
        } else {
            kSpeakerApriltagPose = VisionConstants.BlueSpeakerPose;
        }
        return thetaController.calculate(swervePose.getRotation().getDegrees() ,kSpeakerApriltagPose.minus(swervePose.getTranslation()).getAngle().getDegrees());
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