package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;

import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;

import frc.robot.subsystems.CommandSwerveDrivetrain;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(70.0);

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    public static final CANBus kCANBus = new CANBus("Swerve");

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.76);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5;

    public static final double kDriveGearRatio = 5.357142857142857;
    public static final double kSteerGearRatio = 21.428571428571427;
    private static final Distance kWheelRadius = Inches.of(2); //RADIUS DIAMTER DEGIL

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 0;

    // These are only used for simulation
    private static final double kSteerInertia = 0.01;
    private static final double kDriveInertia = 0.01;
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.25);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.25);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withCANcoderInitialConfigs(cancoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);


    public static final double kSagSolArasi = 0.51435;

    // Front Left
    private static final int kFrontLeftDriveMotorId = 13;
    private static final int kFrontLeftSteerMotorId = 14;
    private static final int kFrontLeftEncoderId = 51;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.923583984375);
    private static final boolean kFrontLeftSteerMotorInverted  = true;
    private static final boolean kFrontLeftCANcoderInverted = false;

    private static final Distance kFrontLeftXPos = Meters.of(0.259428);
    private static final Distance kFrontLeftYPos = Meters.of(kSagSolArasi / 2);

    // Front Right
    private static final int kFrontRightDriveMotorId = 4;
    private static final int kFrontRightSteerMotorId = 9;
    private static final int kFrontRightEncoderId = 52;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.322265625);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightCANcoderInverted = false;

    private static final Distance kFrontRightXPos = Meters.of(0.259428);
    private static final Distance kFrontRightYPos = Meters.of(-kSagSolArasi / 2);

    // Back Left
    private static final int kBackLeftDriveMotorId = 12;
    private static final int kBackLeftSteerMotorId = 11;
    private static final int kBackLeftEncoderId = 50;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.61767578125);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftCANcoderInverted = false;

    private static final Distance kBackLeftXPos = Meters.of(-0.102523);
    private static final Distance kBackLeftYPos = Meters.of(kSagSolArasi / 2);

    // Back Right
    private static final int kBackRightDriveMotorId = 62;
    private static final int kBackRightSteerMotorId = 5;
    private static final int kBackRightEncoderId = 53;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.260498046875);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightCANcoderInverted = false;

    private static final Distance kBackRightXPos = Meters.of(-0.102523);
    private static final Distance kBackRightYPos = Meters.of(-kSagSolArasi / 2);


    public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftCANcoderInverted);
    public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightCANcoderInverted);
    public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftCANcoderInverted);
    public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightCANcoderInverted);

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,.
     */
    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }
}
