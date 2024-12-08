package frc.robot.subsystems.apriltagvision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;

public class RealPhotonVision implements AprilTagVisionIO{
    protected final PhotonCamera camera;
    protected final PhotonPoseEstimator poseEstimator;
    protected final Transform3d robotToCamera;

    public RealPhotonVision(String name, Transform3d robotToCamera){
        camera = new PhotonCamera(name);

        poseEstimator = new PhotonPoseEstimator(
                        VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var result : camera.getAllUnreadResults()) {
            if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();
                
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                tagIds.addAll(multitagResult.fiducialIDsUsed);

                poseObservations.add(new PoseObservation(
                    poseEstimator.update(result).orElse(null),
                    multitagResult.estimatedPose.ambiguity,
                    multitagResult.fiducialIDsUsed.size(),
                    totalTagDistance / multitagResult.fiducialIDsUsed.size()
                ));
            }
        }

        inputs.poseObservations = poseObservations.toArray(new PoseObservation[poseObservations.size()]);
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
          inputs.tagIds[i++] = id;
        }
    }
}
