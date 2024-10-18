package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.LEDSubsystem;

public class GoToFeedPosition extends Command {
    private Command pathCommand;
    private Pose2d targetPose;
    private final LEDSubsystem ledSubsystem;

    public GoToFeedPosition(LEDSubsystem ledSubsystem){
        this.ledSubsystem = ledSubsystem;
    }

    @Override
    public void initialize(){
        if (GlobalVariables.getInstance().alliance == Alliance.Red) {
            targetPose = new Pose2d(6.9, 0.92, Rotation2d.fromDegrees(33.3)); //TUNELANACAK
        } else {
            targetPose = new Pose2d(9.68, 0.92, Rotation2d.fromDegrees(146.67)); //TUNELANACAK
        }
            // Create the constraints to use pathfinding
            PathConstraints constraints = AutoConstants.kPathConstraints;

            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            pathCommand = AutoBuilder.pathfindToPose(
                    targetPose,
                    constraints,
                    0.0, // Goal end velocity in meters/sec
                    0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
            );
        pathCommand.schedule();
        ledSubsystem.isAutodrive = true;
    }

    @Override
    public void execute(){        
            pathCommand.execute();
    }

    @Override
    public void end(boolean interrupted){
        ledSubsystem.isAutodrive = false;
    }

    @Override
    public boolean isFinished(){
        if (pathCommand.isFinished()) {
            ledSubsystem.isAutodrive = false;
            return true;
        }
        return false;
    }
}
