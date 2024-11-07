package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.VisionConstants;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.List;

public class OV9281 extends SubsystemBase{
    private final PhotonCamera camera;
    private PhotonPipelineResult result;
    private final PhotonPoseEstimator photonEstimator;
    private double lastEstTimestamp = 0;
    private EstimatedRobotPose visionEst;
    private final CommandSwerveDrivetrain drivetrain;
    private final String cameraName;
    public List<Pose3d> poseList = new ArrayList<>();

    public OV9281(CommandSwerveDrivetrain drivetrain, String cameraName, Transform3d robotToCamera) {
        this.drivetrain = drivetrain;
        this.cameraName = cameraName;

        camera = new PhotonCamera(cameraName);

        result = camera.getLatestResult();

        photonEstimator =
                new PhotonPoseEstimator(
                        VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        GlobalVariables.getInstance().isDetected(isTargetValid(), cameraName);
    }

    @Override
    public void periodic(){
        result = camera.getLatestResult();

        GlobalVariables.getInstance().isDetected(isTargetValid(), cameraName);

        visionEst = getEstimatedGlobalPose().orElse(null);

        if (visionEst!= null) {
        var photonPoseEst = visionEst.estimatedPose;
        var estPose2d = photonPoseEst.toPose2d();
        var estStdDevs = getEstimationStdDevs(visionEst.estimatedPose.toPose2d());
        drivetrain.addVisionMeasurement(estPose2d, visionEst.timestampSeconds, estStdDevs);
        }

        poseList.clear();
        for (var tgt : result.targets) {
            poseList.add(VisionConstants.kTagLayout.getTagPose(tgt.getFiducialId()).orElse(null));
        }

        Logger.recordOutput("Vision/"+cameraName+"/Connected", camera.isConnected());
        Logger.recordOutput("Vision/"+cameraName+"/Poses", poseList.toArray(Pose3d[]::new));
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        var targets = result.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public boolean isTargetValid() {
        return result.hasTargets(); 
    }
}