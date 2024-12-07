// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class SwerveWheelCalibration extends Command {

  private final CommandSwerveDrivetrain drivetrain;

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  private final SwerveRequest.RobotCentric robotOriented = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want r-centric driving in open loop

  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  public SwerveWheelCalibration(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = drivetrain.getState().Pose.getRotation().getRadians();
    accumGyroYawRads = 0.0;

    startWheelPositions = new double[] {
      Units.rotationsToRadians(drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble() / TunerConstants.kDriveGearRatio),
      Units.rotationsToRadians(drivetrain.getModule(1).getDriveMotor().getPosition().getValueAsDouble() / TunerConstants.kDriveGearRatio), 
      Units.rotationsToRadians(drivetrain.getModule(2).getDriveMotor().getPosition().getValueAsDouble() / TunerConstants.kDriveGearRatio),
      Units.rotationsToRadians(drivetrain.getModule(3).getDriveMotor().getPosition().getValueAsDouble() / TunerConstants.kDriveGearRatio)
      };

      omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    drivetrain.setControl(robotOriented.withVelocityX(0)
    .withVelocityY(0)
    .withRotationalRate(omegaLimiter.calculate(4)));

    // Get yaw and wheel positions
    accumGyroYawRads += MathUtil.angleModulus(drivetrain.getState().Pose.getRotation().getRadians() - lastGyroYawRads);
    lastGyroYawRads = drivetrain.getState().Pose.getRotation().getRadians();
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = new double[] {
        Units.rotationsToRadians(drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble() / TunerConstants.kDriveGearRatio),
        Units.rotationsToRadians(drivetrain.getModule(1).getDriveMotor().getPosition().getValueAsDouble() / TunerConstants.kDriveGearRatio), 
        Units.rotationsToRadians(drivetrain.getModule(2).getDriveMotor().getPosition().getValueAsDouble() / TunerConstants.kDriveGearRatio),
        Units.rotationsToRadians(drivetrain.getModule(3).getDriveMotor().getPosition().getValueAsDouble() / TunerConstants.kDriveGearRatio)
    }; //1.95872
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumGyroYawRads * drivetrain.getDriveBaseRadius()) / averageWheelPosition;
}

  @Override
  public void end(boolean interrupted) {
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}