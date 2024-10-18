package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CoupleRatioTool extends Command{

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.PointWheelsAt pointWheelsAt;
    private double start;

    public CoupleRatioTool(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        this.pointWheelsAt = new SwerveRequest.PointWheelsAt();
    }

    @Override
    public void initialize(){
        drivetrain.getDefaultCommand().cancel();
        drivetrain.removeDefaultCommand();
    }

    @Override
    public void execute(){
        drivetrain.applyRequest(() -> pointWheelsAt.withModuleDirection(new Rotation2d(0)));
        start = drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble();
        drivetrain.applyRequest(() -> pointWheelsAt.withModuleDirection(new Rotation2d(90)));
        drivetrain.applyRequest(() -> pointWheelsAt.withModuleDirection(new Rotation2d(180)));
        drivetrain.applyRequest(() -> pointWheelsAt.withModuleDirection(new Rotation2d(270)));
        drivetrain.applyRequest(() -> pointWheelsAt.withModuleDirection(new Rotation2d(360)));
        System.out.println(drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble() - start);
    }

    @Override
    
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
