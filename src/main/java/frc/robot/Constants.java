// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.interpolation.UnivariateInterpolator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
  
  public static final class AutoConstants {
    public static final double kRobotWeight = 47.6;
    public static final double kMOI = 10;

    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kPXYController = 10;
    public static final double kPThetaController = 7;

    public static final PathConstraints kPathConstraints = new PathConstraints(
      kMaxSpeedMetersPerSecond, 3, Math.toRadians(540), Math.toRadians(720));
  }

  public static class VisionConstants {
    public static final Transform3d kRobotToCam1 = //OV9281 001
                new Transform3d(new Translation3d(0.0874014, -0.0479552, 0.61602),
                                new Rotation3d(0, Math.toRadians(-18.05), Math.toRadians(1.5))
                                );
    public static final Transform3d kRobotToCam2 = //OV9281 002
                new Transform3d(new Translation3d(-0.070937, -0.264048, 0.212),
                                new Rotation3d(0, Math.toRadians(-28.125), Math.toRadians(-150))
                                );

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    public static final Translation2d RedSpeakerPose = kTagLayout.getTagPose(4).get().getTranslation().toTranslation2d().minus(new Translation2d(0.1, 0));
    public static final Translation2d BlueSpeakerPose = kTagLayout.getTagPose(7).get().getTranslation().toTranslation2d().plus(new Translation2d(0.1, 0));

    public static final double x_DistanceToSpeaker[] = {0   , 1.3 , 1.5 , 2   , 2.5 , 3   , 100000};
    public static final double y_ArmAngle[] =          {31  , 31  , 22  , 16  , 10  , 6.7   , 0};
    public static final UnivariateInterpolator angleInterpolator = new SplineInterpolator();
    public static final UnivariateFunction angleFunction = angleInterpolator.interpolate(x_DistanceToSpeaker, y_ArmAngle);

    public static final double kFeedAngle = 4;
    public static final double kAmpAngle = 22;

    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    public static final double kMaxAmbiguity = 0.3;
    public static final double kMaxZError = 0.75;
  }

  public static class PIDConstants{
    public static final double kPLimeLightRotate = 0.07;
    public static final double kDLimeLightRotate = 0;

    public static final double kPObjectRotate = 0.105;
    public static final double kDObjectRotate = 0.015;

    public static final double kPObjectSpeed = 0.1;
  }

  public static class ShooterConstants{
    
    public static final double kGearRatio = 1.0 / 11.357; // CANCODERIN GEAR RATIO
    public static final double kPivotMotorRot2Rad = kGearRatio * 2 * Math.PI;
    public static final double kMinShooterAngleRad = 0;
    public static final double kMaxShooterAngleRad = Math.toRadians(39);

    public static final int kPivotMotorId = 3;
    public static final int kAbsoluteEncoderId = 54;

    public static final double kAbsoluteEncoderOffset = 0.011;
    public static final boolean kPivotMotorReversed = true;

    public static final double kAngleP = 1;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;
    public static final double kAngleToleranceRad = Math.toRadians(0.05);

    ////////////////////////////////////////////////////////////////////////////////
    
    
    public static final int kShooterMotorLeftId = 7;
    public static final int kShooterMotorRightId = 8;    

  
    public static final TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();

    // set slot 0 gains
    private static final Slot0Configs left_slot0Configs;

    static {
      left_slot0Configs = leftMotorConfig.Slot0;
      left_slot0Configs.kS = 0.27698; // Add 0.25 V output to overcome static friction
      left_slot0Configs.kV = 0.12961; // A velocity target of 1 rps results in 0.12 V output
      left_slot0Configs.kA = 0.019564; // An acceleration of 1 rps/s requires 0.01 V output
      left_slot0Configs.kP = 0.20423; // An error of 1 rps results in 0.11 V output
      left_slot0Configs.kI = 0; // no output for integrated error
      left_slot0Configs.kD = 0; // no output for error derivative 
    }

    // set Motion Magic Velocity settings
    private static final MotionMagicConfigs left_motionMagicConfigs;

    static {
      left_motionMagicConfigs = leftMotorConfig.MotionMagic;
      left_motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
      left_motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
    }

    public static final TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

    // set slot 0 gains
    private static final Slot0Configs right_slot0Configs;

    static {
      right_slot0Configs = rightMotorConfig.Slot0;
      right_slot0Configs.kS = 0.32527; // Add 0.25 V output to overcome static friction
      right_slot0Configs.kV = 0.12874; // A velocity target of 1 rps results in 0.12 V output
      right_slot0Configs.kA = 0.013443; // An acceleration of 1 rps/s requires 0.01 V output
      right_slot0Configs.kP = 0.19824; // An error of 1 rps results in 0.11 V output
      right_slot0Configs.kI = 0; // no output for integrated error
      right_slot0Configs.kD = 0; // no output for error derivative 
    }

    // set Motion Magic Velocity settings
    private static final MotionMagicConfigs right_motionMagicConfigs;

    static {
      right_motionMagicConfigs = rightMotorConfig.MotionMagic;
      right_motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
      right_motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
    }

    public static final double kSpeakerSpeedLeft = 64.3;
    public static final double kSpeakerSpeedRight = 52.3;

    public static final double kAmpSpeedLeft = 21.5;
    public static final double kAmpSpeedRight = 21.5;
    
    public static final double kVoltageCompensation = 10;
  }

  public static class IntakextenderConstants{
    public static final int kIntakeMotorId = 10;
    public static final boolean kIntakeMotorReversed = false;
    public static final double kIntakeMotorSpeed = 0.65;
    public static final double kIntakeDeadband = 0.3;


    public static final int kExtenderMotorId = 62;
    public static final boolean kExtenderMotorReversed = false;
    public static final double kExtenderSpeed = 0.15;
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
