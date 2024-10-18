package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase{
    private final PhotonCamera camera;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private double tx;

    public ObjectDetection() {        
        camera = new PhotonCamera("ar0234");

        result = camera.getLatestResult();
    }

    public double getTX() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            tx = target.getYaw();
        } else {
            tx = 0;
        }
        return tx;
    }
}
