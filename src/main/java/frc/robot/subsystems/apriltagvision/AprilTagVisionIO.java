package frc.robot.subsystems.apriltagvision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface AprilTagVisionIO {
    @AutoLog
    public static class AprilTagVisionIOInputs {
        public boolean connected = false;
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
    }

    public static record PoseObservation(
      EstimatedRobotPose estPose,
      double ambiguity,
      int tagCount,
      double averageTagDistance) {}

    public default void updateInputs(AprilTagVisionIOInputs inputs) {}
} 
