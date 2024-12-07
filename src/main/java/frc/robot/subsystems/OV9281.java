package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.ArrayList;
import java.util.List;

public class OV9281 extends SubsystemBase{
    private final PhotonCamera camera;
    private List<PhotonPipelineResult> results;
    private final PhotonPoseEstimator photonEstimator;
    private EstimatedRobotPose visionEst;
    private final CommandSwerveDrivetrain drivetrain;
    private final String cameraName;
    public List<Pose3d> poseList = new ArrayList<>();
    private final Pose3d emptyPose = new Pose3d();
    private final boolean useSim = Robot.isSimulation();
    private final VisionSystemSim visionSim;
    private Matrix<N3, N1> curStdDevs;

    public OV9281(CommandSwerveDrivetrain drivetrain, String cameraName, Transform3d robotToCamera) {
        this.drivetrain = drivetrain;
        this.cameraName = cameraName;

        camera = new PhotonCamera(cameraName);

        if(useSim) {
            visionSim = new VisionSystemSim("01");
            visionSim.addAprilTags(VisionConstants.kTagLayout);
            SimCameraProperties cameraProperties = new SimCameraProperties();
            cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(79));
            cameraProperties.setFPS(25);
            PhotonCameraSim camerasim = new PhotonCameraSim(camera, cameraProperties);
            visionSim.addCamera(camerasim, robotToCamera);
        } else {
            visionSim = null;
        }
        photonEstimator =
                new PhotonPoseEstimator(
                        VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        GlobalVariables.getInstance().isDetected(isTargetValid(), cameraName);
    }

    @Override
    public void periodic(){
        if (useSim) {
            visionSim.update(drivetrain.getState().Pose);
        }

        results = camera.getAllUnreadResults();

        GlobalVariables.getInstance().isDetected(isTargetValid(), cameraName);

        visionEst = getEstimatedGlobalPose().orElse(null);

        if (visionEst!= null) {
        var photonPoseEst = visionEst.estimatedPose;
        var estPose2d = photonPoseEst.toPose2d();
        var estStdDevs = getEstimationStdDevs();
        drivetrain.addVisionMeasurement(estPose2d, visionEst.timestampSeconds, estStdDevs);
        Logger.recordOutput("Vision/"+cameraName+"/Current Estimation", visionEst.estimatedPose);
        } else {
            Logger.recordOutput("Vision/"+cameraName+"/Current Estimation", emptyPose);
        }

        poseList.clear();
        for (var tgt : results.get(results.size() - 1).getTargets()) {
            poseList.add(VisionConstants.kTagLayout.getTagPose(tgt.getFiducialId()).orElse(null));
        }

        Logger.recordOutput("Vision/"+cameraName+"/Connected", camera.isConnected());
        Logger.recordOutput("Vision/"+cameraName+"/Poses", poseList.toArray(Pose3d[]::new));
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : results) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }
        }
        return visionEst;
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public boolean isTargetValid() {
        return results.get(results.size() - 1).hasTargets(); 
    }

     public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}