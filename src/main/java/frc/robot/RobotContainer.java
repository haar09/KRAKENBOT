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

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AmpMechanismCmd;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.DriveToNote;
import frc.robot.commands.FeedAuto;
import frc.robot.commands.GoToFeedPosition;
import frc.robot.commands.ShooterAuto;
import frc.robot.commands.ShooterShoot;
import frc.robot.commands.SwerveWheelCalibration;
import frc.robot.commands.AutoCommands.PreSpeedup;
import frc.robot.commands.AutoCommands.SpeakerShoot;
import frc.robot.commands.SetAngle.AmpAngle;
import frc.robot.commands.SetAngle.ShooterAutoAngle;
import frc.robot.commands.SetAngle.ShooterSetAngle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.OV9281;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.ampMechanism.AmpMechanism;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;
import frc.robot.subsystems.rollers.extender.Extender;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1); // My joystick

  private void configureBindings() {
    updateControlStyle();

    joystick.y().onTrue(Commands.runOnce(drive::stopWithX, drive));

    joystick.leftTrigger(IntakextenderConstants.kIntakeDeadband).whileTrue(rollers.setStateCommand(RollerState.FLOOR_INTAKING));
    joystick.a().whileTrue(rollers.setStateCommand(RollerState.EJECTING));
    joystick.x().whileTrue(rollers.setStateCommand(RollerState.STUCK_INTAKING));
    joystick.x().and(joystick.a()).whileTrue(rollers.setStateCommand(RollerState.STUCK_EJECTING));

    joystick.b().or(joystick.rightStick()).onTrue(new DriveToNote(drivetrain, objectDetection, joystick.getHID(), ledSubsystem, beamBreak)
    .beforeStarting(rollers.setStateCommand(RollerState.FLOOR_INTAKING)));

    joystick.leftBumper().onTrue(runOnce(() -> controlMode = 1).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));
    joystick.leftBumper().onFalse(runOnce(() -> controlMode = 0).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));

    joystick.rightBumper().onTrue(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * SlowSpeed).andThen(() -> AngularRate = SlowAngularRate)
    .andThen(() -> drive.withDeadband(0.0).withRotationalDeadband(0.0)).andThen(
      () -> robotOriented.withDeadband(0.0).withRotationalDeadband(0.0)));
    joystick.rightBumper().onFalse(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps).andThen(() -> AngularRate = MaxAngularRate)
    .andThen(() -> drive.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1))
    .andThen(() -> robotOriented.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)));

    joystick.rightTrigger(0.3).whileTrue(new ShooterAuto(shooter, rollers, drivetrain, shooterPivot, joystick.getHID()))
    .onTrue(runOnce(() -> controlMode = 2).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));

    joystick.rightTrigger(0.3).onFalse(runOnce(() -> controlMode = 0).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));

    joystick.pov(180).onTrue(new GoToFeedPosition(ledSubsystem));

    joystick.pov(0).whileTrue(new FeedAuto(shooter, rollers, shooterPivot, joystick.getHID()))
    .onTrue(runOnce(() -> controlMode = 4).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));
    joystick.pov(0).onFalse(runOnce(() -> controlMode = 0).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));

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
    joystick.leftStick().whileTrue(drivetrain.runDriveSlipTest());*/
    joystick.back().whileTrue(new SwerveWheelCalibration(drivetrain));
    // HALILI TESTLER BİTİŞ


    // reset the field-centric heading on menu button
    joystick.start().onTrue(drive.resetFieldOriented());

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle",
            () -> drive.modules[0].getAngle().getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity",
            () -> drive.modules[0].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Front Right Angle",
            () -> drive.modules[1].getAngle().getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity",
            () -> drive.modules[1].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Back Left Angle", () -> drive.modules[2].getAngle().getRadians(),
            null);
        builder.addDoubleProperty("Back Left Velocity",
            () -> drive.modules[2].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Back Right Angle",
            () -> drive.modules[3].getAngle().getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity",
            () -> drive.modules[3].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Robot Angle", () -> drive.getRotation().getRadians(), null);
      }
    });
    
    operator.x().whileTrue(new AmpAngle(shooterPivot));
    operator.a().whileTrue(new ShooterSetAngle(shooterPivot));
    operator.y().whileTrue(new ShooterAutoAngle(shooterPivot));

    operator.b().onTrue(runOnce(() -> controlMode = 2).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));
    operator.b().onFalse(runOnce(() -> controlMode = 0).andThen(() -> updateControlStyle()).withName("controlStyleUpdate"));

    operator.rightBumper().whileTrue(new ShooterShoot(
      () -> operator.getRightTriggerAxis(),
      shooter, rollers, false, operator.getHID()));

    operator.leftBumper().whileTrue(new ShooterShoot(
      () -> operator.getRightTriggerAxis(),
      shooter, rollers, true, operator.getHID())); //l1 ve r2
    }

  private final LEDSubsystem ledSubsystem;
  private final Extender extender;
  private final Intake intake;
  private final Rollers rollers;
  private final Shooter shooter;
  private final ObjectDetection objectDetection;
  private final AmpMechanism ampMechanism;
  private final Climb climb;
  private final Pivot shooterPivot;
  private final BeamBreak beamBreak;
  private final Drive drive;

  private LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    drive = Drive.create(new GyroIOPigeon2(true),
      new ModuleIOTalonFX(0),
      new ModuleIOTalonFX(1),
      new ModuleIOTalonFX(2),
      new ModuleIOTalonFX(3));
    ledSubsystem = new LEDSubsystem();
    extender = Extender.create();
    intake = Intake.create();
    beamBreak = new BeamBreak();
    rollers = new Rollers(extender, intake, beamBreak, ledSubsystem);
    shooter = Shooter.create(ledSubsystem);    
    shooterPivot = Pivot.create();
    objectDetection = new ObjectDetection();
    ampMechanism = AmpMechanism.create();
    climb = Climb.create();

    new OV9281(drivetrain, "Arducam_OV9281_USB_Camera_001", VisionConstants.kRobotToCam1);
    new OV9281(drivetrain, "Arducam_OV9281_USB_Camera_002", VisionConstants.kRobotToCam2);

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

    NamedCommands.registerCommand("ShootToSpeakerAuto", new SpeakerShoot(shooter, shooterPivot, rollers, 0));
    NamedCommands.registerCommand("ShootToSpeaker0m", new SpeakerShoot(shooter, shooterPivot, rollers, VisionConstants.y_ArmAngle[1]));
    NamedCommands.registerCommand("IntakeIn", rollers.setStateCommand(RollerState.FLOOR_INTAKING));
    NamedCommands.registerCommand("AmpSpeed", new PreSpeedup(shooter));

    configureBindings();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  private void updateControlStyle() {
    SmartDashboard.putNumber("Control Mode", controlMode);
    Logger.recordOutput("Drive/Control Mode", controlMode);
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
    drivetrain.setDefaultCommand(drivetrain.applyRequest(controlStyle).ignoringDisable(true).withName("Swerve Command "+controlMode));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /*private Command rumbleDriverController() {
    return Commands.startEnd(
      () -> joystick.getHID().setRumble(RumbleType.kBothRumble, 1),
      () -> joystick.getHID().setRumble(RumbleType.kBothRumble, 0));
  }*/
}
