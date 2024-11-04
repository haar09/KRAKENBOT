// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AmpMechanismCmd;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.DriveToNote;
import frc.robot.commands.FeedAuto;
import frc.robot.commands.GoToFeedPosition;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShooterAuto;
import frc.robot.commands.ShooterShoot;
import frc.robot.commands.SwerveWheelCalibration;
import frc.robot.commands.AutoCommands.IntakeIn;
import frc.robot.commands.AutoCommands.PreSpeedup;
import frc.robot.commands.AutoCommands.SpeakerShoot;
import frc.robot.commands.SetAngle.AmpAngle;
import frc.robot.commands.SetAngle.ShooterAutoAngle;
import frc.robot.commands.SetAngle.ShooterSetAngle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AmpMechanism;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.OV9281;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private double SlowSpeed = 0.1;
  private double SlowAngularRate = 0.25 * Math.PI;
  private double AngularRate = MaxAngularRate;
  private int controlMode = 0;
  
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

  private final SwerveRequest.RobotCentric robotOriented = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want r-centric driving in open loop

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(drivetrain.getState());
  private Supplier<SwerveRequest> controlStyle;

  private void configureBindings() {
    updateControlStyle();

    joystick.y().whileTrue(drivetrain.applyRequest(() -> brake));

    joystick.b().or(joystick.rightStick()).onTrue(new DriveToNote(drivetrain, objectDetection, joystick.getHID(), intake, extender, ledSubsystem));

    joystick.leftBumper().onTrue(runOnce(() -> controlMode = 1).andThen(() -> updateControlStyle()));
    joystick.leftBumper().onFalse(runOnce(() -> controlMode = 0).andThen(() -> updateControlStyle()));

    joystick.rightBumper().onTrue(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * SlowSpeed).andThen(() -> AngularRate = SlowAngularRate)
    .andThen(() -> drive.withDeadband(0.0).withRotationalDeadband(0.0)).andThen(
      () -> robotOriented.withDeadband(0.0).withRotationalDeadband(0.0)));
    joystick.rightBumper().onFalse(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps).andThen(() -> AngularRate = MaxAngularRate)
    .andThen(() -> drive.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1))
    .andThen(() -> robotOriented.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)));

    joystick.rightTrigger(0.3).whileTrue(new ShooterAuto(shooter, intake, extender, drivetrain, shooterPivot, joystick.getHID()))
    .onTrue(runOnce(() -> controlMode = 2).andThen(() -> updateControlStyle()));

    joystick.rightTrigger(0.3).onFalse(runOnce(() -> controlMode = 0).andThen(() -> updateControlStyle()));

    /*joystick.pov(180).onTrue(new GoToFeedPosition(ledSubsystem));

    joystick.pov(0).whileTrue(new FeedAuto(shooter, intake, extender, shooterPivot, joystick.getHID()))
    .onTrue(runOnce(() -> controlMode = 4).andThen(() -> updateControlStyle()));
    joystick.pov(0).onFalse(runOnce(() -> controlMode = 0).andThen(() -> updateControlStyle()));*/

    joystick.pov(0).whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
    joystick.pov(90).whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
    joystick.pov(180).whileTrue(shooter.sysIdDynamic(Direction.kForward));
    joystick.pov(270).whileTrue(shooter.sysIdDynamic(Direction.kReverse));

    // HALILI TESTLER
    /*
    joystick.pov(0).whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
    joystick.pov(90).whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));
    joystick.pov(180).whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
    joystick.pov(270).whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));
    joystick.pov(0).whileTrue(drivetrain.runSteerQuasiTest(Direction.kForward));
    joystick.pov(90).whileTrue(drivetrain.runSteerQuasiTest(Direction.kReverse));
    joystick.pov(180).whileTrue(drivetrain.runSteerDynamTest(Direction.kForward));
    joystick.pov(270).whileTrue(drivetrain.runSteerDynamTest(Direction.kReverse));
     */
    joystick.leftStick().whileTrue(drivetrain.runDriveSlipTest());
    joystick.back().whileTrue(new SwerveWheelCalibration(drivetrain));
    // HALILI TESTLER BİTİŞ


    // reset the field-centric heading on menu button
    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle",
            () -> drivetrain.getModule(0).getCurrentState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity",
            () -> drivetrain.getModule(0).getCurrentState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle",
            () -> drivetrain.getModule(1).getCurrentState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity",
            () -> drivetrain.getModule(1).getCurrentState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> drivetrain.getModule(2).getCurrentState().angle.getRadians(),
            null);
        builder.addDoubleProperty("Back Left Velocity",
            () -> drivetrain.getModule(2).getCurrentState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle",
            () -> drivetrain.getModule(3).getCurrentState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity",
            () -> drivetrain.getModule(3).getCurrentState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> drivetrain.getState().Pose.getRotation().getRadians(), null);
      }
    });
    
    operator.x().whileTrue(new AmpAngle(shooterPivot));
    operator.a().whileTrue(new ShooterSetAngle(shooterPivot));
    operator.y().whileTrue(new ShooterAutoAngle(shooterPivot));

    operator.b().onTrue(runOnce(() -> controlMode = 2).andThen(() -> updateControlStyle()));
    operator.b().onFalse(runOnce(() -> controlMode = 0).andThen(() -> updateControlStyle()));

    operator.rightBumper().whileTrue(new ShooterShoot(
      () -> operator.getRightTriggerAxis(),
      shooter, intake, extender, false, operator.getHID()));

    operator.leftBumper().whileTrue(new ShooterShoot(
      () -> operator.getRightTriggerAxis(),
      shooter, intake, extender, true, operator.getHID())); //l1 ve r2
    }

  private final LEDSubsystem ledSubsystem;
  private final Extender extender;
  private final Intake intake;
  private final Shooter shooter;
  private final ObjectDetection objectDetection;
  private final AmpMechanism ampMechanism;
  private final Climb climb;
  private final ShooterPivot shooterPivot;

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    ledSubsystem = new LEDSubsystem();
    extender = new Extender();
    intake = new Intake();
    shooter = new Shooter(ledSubsystem);    
    shooterPivot = new ShooterPivot();
    objectDetection = new ObjectDetection();
    ampMechanism = new AmpMechanism();
    climb = new Climb();

    new OV9281(drivetrain);
    new BeamBreak();

    intake.setDefaultCommand(new IntakeCmd(        
      () -> joystick.getLeftTriggerAxis() > IntakextenderConstants.kIntakeDeadband,
      () -> joystick.getHID().getAButton(),
      () -> joystick.getHID().getXButton(),
      intake,
      extender,
      joystick.getHID()
    ));

    ampMechanism.setDefaultCommand(
      new AmpMechanismCmd(
        ampMechanism, 
        () -> -operator.getLeftY()
      )
    );

    climb.setDefaultCommand(
      new ClimbCmd(
      () -> operator.getHID().getPOV() == 0, 
      () -> operator.getHID().getPOV() == 180, 
      climb)
    );

    NamedCommands.registerCommand("ShootToSpeakerAuto", new SpeakerShoot(shooter, shooterPivot, extender, intake, 0));
    NamedCommands.registerCommand("ShootToSpeaker0m", new SpeakerShoot(shooter, shooterPivot, extender, intake, VisionConstants.y_ArmAngle[1]));
    NamedCommands.registerCommand("IntakeIn", new IntakeIn(intake, extender));
    NamedCommands.registerCommand("AmpSpeed", new PreSpeedup(shooter));

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void updateControlStyle() {
    SmartDashboard.putNumber("Control Mode", controlMode);
    switch (controlMode) {
      case 0:
        controlStyle = () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
        .withRotationalRate(-joystick.getRightX() * AngularRate);
        break;
      case 1:
        controlStyle = () -> robotOriented.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
        .withRotationalRate(-joystick.getRightX() * AngularRate);
        break;
      case 2:
        controlStyle = () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
        .withRotationalRate(GlobalVariables.getInstance().getSpeakerAngle(drivetrain.getState().Pose));
        break;
      case 4:
        controlStyle = () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
        .withRotationalRate(GlobalVariables.getInstance().getFeedAngle(drivetrain.getState().Pose));
        break;
    }
    try {
        drivetrain.getDefaultCommand().cancel();
    } catch(Exception e) {}
    drivetrain.setDefaultCommand(drivetrain.applyRequest(controlStyle).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
