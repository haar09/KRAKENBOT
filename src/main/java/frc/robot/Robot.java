// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @SuppressWarnings("resource")
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "KRAKENBOT"); // Set a metadata value
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
    
    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter(Filesystem.getOperatingDirectory().getPath()+"/advantage")); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        //Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
  
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              if (command.getName() != "SequentialCommandGroup"){
                logCommandFunction.accept(command, true);
                }
              });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              if (command.getName() != "SequentialCommandGroup"){
              logCommandFunction.accept(command, false);
              }
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              if (command.getName() != "SequentialCommandGroup"){
                logCommandFunction.accept(command, false);
                }
            });

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("TeleOp", isTeleopEnabled());
    SmartDashboard.putNumber("Battery", RobotController.getBatteryVoltage());
    if (isReal()) {
      var canivoreStatus = CANBus.getStatus("Swerve");
      Logger.recordOutput("CANivore/Status", canivoreStatus.Status.getName());
      Logger.recordOutput("CANivore/Utilization", canivoreStatus.BusUtilization);
      Logger.recordOutput("CANivore/OffCount", canivoreStatus.BusOffCount);
      Logger.recordOutput("CANivore/TxFullCount", canivoreStatus.TxFullCount);
      Logger.recordOutput("CANivore/ReceiveErrorCount", canivoreStatus.REC);
      Logger.recordOutput("CANivore/TransmitErrorCount", canivoreStatus.TEC);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    GlobalVariables.getInstance().alliance = DriverStation.getAlliance().orElse(null);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    GlobalVariables.getInstance().alliance = DriverStation.getAlliance().orElse(null);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

}
