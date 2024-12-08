package frc.robot.subsystems.apriltagvision;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class AprilTagVision extends SubsystemBase{
    private final AprilTagVisionConsumer consumer;
    private final AprilTagVisionIO[] io;
    private final AprilTagVisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public AprilTagVision(AprilTagVisionConsumer consumer, AprilTagVisionIO... io){
        this.consumer = consumer;
        this.io = io;

        this.inputs = new AprilTagVisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new AprilTagVisionIOInputsAutoLogged();
        }

        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
        disconnectedAlerts[i] =
            new Alert(
                "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    @Override
    public void periodic(){
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("AprilTagVision/OV9281_" + Integer.toString(i), inputs[i]);
        }

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = VisionConstants.kTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                tagPoses.add(tagPose.get());
                }
            }

            Logger.recordOutput(
            "AprilTagVision/OV9281_" + Integer.toString(cameraIndex) + "/TagPoses",
            tagPoses.toArray(new Pose3d[tagPoses.size()]));

            for (var observation : inputs[cameraIndex].poseObservations) {
                boolean rejectPose =
                observation.estPose().targetsUsed.size() == 0 // Must have at least one tag
                    || (observation.tagCount() == 1
                        && observation.ambiguity() > VisionConstants.kMaxAmbiguity) // Cannot be high ambiguity
                    || Math.abs(observation.estPose().estimatedPose.getZ())
                        > VisionConstants.kMaxZError // Must have realistic Z coordinate
    
                    // Must be within the field boundaries
                    || observation.estPose().estimatedPose.getX() < 0.0
                    || observation.estPose().estimatedPose.getX() > VisionConstants.kTagLayout.getFieldLength()
                    || observation.estPose().estimatedPose.getY() < 0.0
                    || observation.estPose().estimatedPose.getY() > VisionConstants.kTagLayout.getFieldWidth();

                if (rejectPose) {
                robotPosesRejected.add(observation.estPose().estimatedPose);
                } else {
                robotPosesAccepted.add(observation.estPose().estimatedPose);
                }

                if (rejectPose) {
                    continue;
                }

                double stDevFactor = Math.pow(observation.averageTagDistance(), 2) / observation.tagCount();
                double linearStdDev = VisionConstants.linearStdDevBaseline * stDevFactor;
                double angularStdDev = VisionConstants.angularStdDevBaseline * stDevFactor;

                consumer.accept(observation.estPose().estimatedPose.toPose2d(),
                observation.estPose().timestampSeconds,
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            Logger.recordOutput(
            "AprilTagVision/OV9281_" + Integer.toString(cameraIndex) + "/AcceptedEstimatedPoses",
            robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
            "AprilTagVision/OV9281_" + Integer.toString(cameraIndex) + "/RejectedEstimatedPoses",
            robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));  
        }
    }

  @FunctionalInterface
  public static interface AprilTagVisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
