// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.interpolation.UnivariateInterpolator;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {
  
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kPXYController = 3.2;
    public static final double kPThetaController = 2;

    public static final PathConstraints kPathConstraints = new PathConstraints(
      kMaxSpeedMetersPerSecond, 3, Math.toRadians(540), Math.toRadians(720));
  }

  public static class VisionConstants {
    public static final Transform3d kRobotToCam1 = //OV9281 001
                new Transform3d(new Translation3d(0.874014, -0.479552, 0.61602),
                                new Rotation3d(0, Math.toRadians(-15), 0)
                                );
    public static final Transform3d kRobotToCam2 = //OV9281 002
                new Transform3d(new Translation3d(-0.070937, -0.264048, 0.212),
                                new Rotation3d(0, Math.toRadians(-28.1), Math.toRadians(-150))
                                );

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public static final double x_DistanceToSpeaker[] = {0   , 1.3 , 1.5 , 2   , 2.5 , 3   , 3.2 , 100000};
    public static final double y_ArmAngle[] =          {29  , 29  , 22  , 14.5, 6.65, 2.5 , 0   , 0};
    public static final UnivariateInterpolator angleInterpolator = new SplineInterpolator();
    public static final UnivariateFunction angleFunction = angleInterpolator.interpolate(x_DistanceToSpeaker, y_ArmAngle);

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.3, 0.3, 5);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.25, 0.25, 3);
  }

  public static class PIDConstants{
    public static final double kPTurning = 0.45;
    public static final double kDTurning = 0.001;

    public static final double kPLimeLightRotate = 0.04;
    public static final double kDLimeLightRotate = 0.01;

    public static final double kPObjectRotate = 0.08;
    public static final double kDObjectRotate = 0.0015;

    public static final double kP180Rotate = 0.045;
    public static final double kD180Rotate = 0.00001;
  }

  public static class ShooterConstants{
    
    public static final double kGearRatio = 1.0 / 11.357; // CANCODERIN GEAR RATIO
    public static final double kPivotMotorRot2Rad = kGearRatio * 2 * Math.PI;
    public static final float kMinShooterAngle = 0;
    public static final float kMaxShooterAngle = 39;

    public static final int kPivotMotorId = 3;
    public static final int kAbsoluteEncoderId = 54;

    public static final double kAbsoluteEncoderOffset = 0;
    public static final boolean kPivotMotorReversed = true;

    public static final double kAngleP = 1;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;
    public static final double kAngleToleranceRad = Math.toRadians(0.05);

    ////////////////////////////////////////////////////////////////////////////////
    
    
    public static final int kShooterMotorLeftId = 7;
    public static final int kShooterMotorRightId = 8;    
  
    public static final boolean kShooterMotorLeftReversed = false;
    public static final boolean kShooterMotorRightReversed = true; 
  
    public static final double kSpeakerSpeedLeft = 0.9;
    public static final double kSpeakerSpeedRight = 0.75;

    public static final double kAmpSpeedLeft = 0.35;
    public static final double kAmpSpeedRight = 0.35;
    
    public static final double kVoltageCompensation = 10;
  }

  public static class IntakextenderConstants{
    public static final int kIntakeMotorId = 10;
    public static final boolean kIntakeMotorReversed = false;
    public static final double kIntakeMotorSpeed = 0.65;
    public static final double kIntakeDeadband = 0.3;


    public static final int kExtenderMotorId = 2;
    public static final boolean kExtenderMotorReversed = false;
    public static final double kExtenderSpeed = 0.11;
    public static final double kExtenderBackSpeed = 0.3;
  }

  public static class ClimbConstants{
    public static final int kAmpMechanismMotorId = 6;
    public static final boolean kAmpMechanismMotorReversed = false;
    public static final double kAmpMechanismMotorMultiplier = 0.5;

    public static final int kClimbMotorId = 1;
    public static final boolean kClimbMotorReversed = true;
  }
} 
