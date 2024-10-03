package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.ModifiedSignalLogger;
import frc.robot.util.SwerveVoltageRequest;

import static edu.wpi.first.units.Units.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private GenericEntry resetPigeon = Shuffleboard.getTab("TabName")
    .add("Reset Pigeon", false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .getEntry(); 


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        if (resetPigeon.getBoolean(false)) {
            this.getPigeon2().setYaw(0);
            resetPigeon.setBoolean(false);
        }
    }

    private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);

    private SysIdRoutine m_driveSysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
            null,
            this));

    private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(false);

    private SysIdRoutine m_steerSysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> setControl(steerVoltageRequest.withVoltage(volts.in(Volts))),
            null,
            this));

    private SysIdRoutine m_slipSysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null, ModifiedSignalLogger.logState()),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
            null,
            this));
    
    public Command runDriveQuasiTest(Direction direction)
    {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }

    public Command runSteerQuasiTest(Direction direction)
    {
        return m_steerSysIdRoutine.quasistatic(direction);
    }

    public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.dynamic(direction);
    }

    public Command runDriveSlipTest()
    {
        return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }
}
