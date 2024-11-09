package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class ObjectDetection extends SubsystemBase{
    private final PhotonCamera camera;
    private PhotonPipelineResult result;
    private double tx;
    private boolean hasTargets;

    public ObjectDetection() {        
        camera = new PhotonCamera("ar0234");

        result = camera.getLatestResult();
        hasTargets = false;
        tx = 0;
    }

    @Override
    public void periodic(){
        result = camera.getLatestResult();
        hasTargets = result.hasTargets();
        if (result.hasTargets()) {
            tx = result.getBestTarget().getYaw();
        } else {
            tx = 0;
        }

        Logger.recordOutput("Vision/ar0234/Target Valid", hasTargets);
        Logger.recordOutput("Vision/ar0234/Object TX", tx);
        Logger.recordOutput("Vision/ar0234/Connected", camera.isConnected());
    }

    public double getTX() {
        return tx;
    }

    public boolean targetValid() {
       return hasTargets;
    }
}
