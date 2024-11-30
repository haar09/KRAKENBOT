// Developed by Reza from Team Spyder 1622

package frc.robot.util;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class NEO {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;

  private SparkAbsoluteEncoder remoteAbsoluteEncoder;

  private ControlLoopType controlType = ControlLoopType.PERCENT;
  private double targetPosition = 0;
  private double targetVelocity = 0;

  private boolean positionPIDWrappingEnabled = false;
  private double positionPIDWrappingMaxInput = 0;
  private double positionPIDWrappingMinInput = 0;

  private double kS = 0;

  /**
   * Creates a new NEO motor
   *
   * @param id CANID of the SparkMax the NEO is connected to.
   */
  public NEO(int id) {
    this(id, false);
  }

  /**
   * Creates a new NEO motor
   *
   * @param id CANID of the SparkMax the NEO is connected to.
   * @param mode The idle mode of the motor. If true, the motor will brake when not powered. If
   *     false, the motor will coast when not powered.
   */
  public NEO(int id, IdleMode mode) {
    this(id, false, mode, 80);
  }

  /**
   * Creates a new NEO motor
   *
   * @param id CANID of the SparkMax the NEO is connected to.
   * @param reversed Whether the motor is reversed or not.
   * @param mode The idle mode of the motor. If true, the motor will brake when not powered. If
   *     false, the motor will coast when not powered.
   */
  public NEO(int id, boolean reversed, IdleMode mode) {
    this(id, reversed, mode, 80);
  }

  /**
   * Creates a new NEO motor
   *
   * @param id CANID of the SparkMax the NEO is connected to.
   * @param reversed Whether the motor is reversed or not.
   * @param mode The idle mode of the motor. If true, the motor will brake when not powered. If
   *     false, the motor will coast when not powered.
   * @param currentLimit The current limit of the motor in amps.
   */
  public NEO(int id, boolean reversed, IdleMode mode, int currentLimit) {
    motor = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);

    motor.restoreFactoryDefaults();
    Timer.delay(0.050);

    // If a parameter set fails, this will add more time to alleviate any bus
    // traffic default is 20ms
    motor.setCANTimeout(250);

    setInverted(reversed);
    setIdleMode(mode);
    setCurrentLimit(currentLimit);

    encoder = motor.getEncoder();
    pidController = motor.getPIDController();
    
    Timer.delay(0.5);
    burn();

    motor.setCANTimeout(50);
  }

  /**
   * Creates a new NEO motor
   *
   * @param id CANID of the SparkMax the NEO is connected to.
   * @param reversed Whether the motor is reversed or not.
   */
  public NEO(int id, boolean reversed) {
    this(id, reversed, IdleMode.kCoast, 80);
  }

  public boolean isConnected() {
    return motor.getFirmwareVersion() != 0;
  }

  public double getAppliedVoltage() {
    return motor.getBusVoltage() * motor.getAppliedOutput();
  }

  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  /**
   * Sets the NEO to brake or coast mode.
   *
   * @param mode Idle mode, either brake mode or coast mode.
   */
  public void setIdleMode(IdleMode mode) {
    motor.setIdleMode(mode);
  }

  public IdleMode getIdleMode() {
    return motor.getIdleMode();
  }

  /**
   * Sets the target position for the NEO.
   *
   * @param position Position to set the NEO to in rotations.
   * @param arbitraryFeedForward Arbitrary feed forward to add to the motor output.
   */
  public void setTargetPosition(double position, double arbitraryFeedForward, int slot) {
    if (Robot.isReal()) {
      pidController.setReference(
          position,
          ControlType.kPosition,
          slot,
          arbitraryFeedForward + ((Math.signum(position - getPosition()) * kS)),
          SparkPIDController.ArbFFUnits.kVoltage);
    }
    targetPosition = position;
    controlType = ControlLoopType.POSITION;
  }

  public void setTargetPosition(double position, double arbitraryFeedForward) {
    setTargetPosition(position, arbitraryFeedForward, 0);
  }

  public void setTargetPosition(double position) {
    setTargetPosition(position, 0, 0);
  }

  /**
   * Sets the target velocity for the NEO.
   *
   * @param velocity Velocity to set the NEO to in rotations per minute.
   */
  public void setTargetVelocity(double velocity) {
    setTargetVelocity(velocity, 0, 0);
  }

  /**
   * Sets the target velocity for the NEO.
   *
   * @param velocity Velocity to set the NEO to in rotations per minute.
   * @param arbitraryFeedForward Arbitrary feed forward to add to the motor output.
   */
  public void setTargetVelocity(double velocity, double arbitraryFeedForward, int slot) {
    if (velocity == 0) {
      setVoltage(0);
    } else {
      pidController.setReference(
          velocity,
          ControlType.kVelocity,
          slot,
          arbitraryFeedForward + (Math.signum(velocity) * kS),
          SparkPIDController.ArbFFUnits.kVoltage);
    }
    targetVelocity = velocity;
    controlType = ControlLoopType.VELOCITY;
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void set(double percent) {
    setVoltage(percent * RobotController.getBatteryVoltage());
    controlType = ControlLoopType.PERCENT;
  }

  /**
   * Sets the NEO to follow another NEO.
   *
   * @param host The host NEO to follow. This NEO will mirror the host NEO's voltage outputs.
   */
  public void follow(NEO host) {
    follow(host, false);
  }

  /**
   * Sets the NEO to follow another NEO.
   *
   * @param host The host NEO to follow. This NEO will mirror the host NEO's voltage outputs.
   * @param inverted Sets the NEO to be inverted.
   */
  public void follow(NEO host, boolean inverted) {
    motor.follow(host.getMotor(), inverted);
  }

  public void setPositionConversionFactor(double ratio) {
    encoder.setPositionConversionFactor(ratio);
  }

  public void setVelocityConversionFactor(double ratio) {
    encoder.setVelocityConversionFactor(ratio);
  }

  public void setMeasurementPeriod(int periodMs) {
    encoder.setMeasurementPeriod(periodMs);
  }

  public void setAverageDepth(int depth) {
    encoder.setAverageDepth(depth);
  }

  public void tick() {
    if (Robot.isSimulation() && controlType == ControlLoopType.POSITION ) {
      double currentPosition = getPosition();

      if (remoteAbsoluteEncoder != null) {
        currentPosition = remoteAbsoluteEncoder.getPosition();
      }

      double error = targetPosition - currentPosition;

      if (positionPIDWrappingEnabled) {
        double errorBound = (positionPIDWrappingMaxInput - positionPIDWrappingMinInput) / 2.0;
        error = MathUtil.inputModulus(error, -errorBound, errorBound);
      }

      setVoltage(pidController.getP() * error);
    } else if (Robot.isSimulation() && DriverStation.isDisabled()) {
      setVoltage(0);
    }
  }

  /**
   * Gets the position of the NEO in rotations.
   *
   * @return The position of the NEO in rotations relative to the last 0 position.
   */
  public double getPosition() {
    double pos = encoder.getPosition();

    if (Robot.isSimulation() && controlType == ControlLoopType.VELOCITY) {
      pos /= encoder.getVelocityConversionFactor();
    }

    return pos;
  }

  /**
   * Gets the velocity of the NEO in rotations per minute.
   *
   * @return The instantaneous velocity of the NEO in rotations per minute.
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Gets the target position of the NEO in rotations.
   *
   * @return The target position of the NEO in rotations.
   */
  public double getTargetPosition() {
    return targetPosition;
  }

  /**
   * Gets the target velocity of the NEO in rotations per minute.
   *
   * @return The target velocity of the NEO in rotations per minute.
   */
  public double getTargetVelocity() {
    return targetVelocity;
  }

  /**
   * Gets the current of the NEO in amps.
   *
   * @return The current of the NEO in amps.
   */
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Sets the current encoder position to whatever is specified in rotations.
   *
   * @param position The position to set the encoder to in rotations.
   */
  public void setEncoderPosition(double position) {
    encoder.setPosition(position);
  }

  public void setInverted(boolean inverted) {
    motor.setInverted(inverted);
  }

  public void setPositionPIDWrappingRange(double minInput, double maxInput) {
    pidController.setPositionPIDWrappingEnabled(true);
    pidController.setPositionPIDWrappingMaxInput(maxInput);
    pidController.setPositionPIDWrappingMinInput(minInput);

    positionPIDWrappingEnabled = true;
    positionPIDWrappingMaxInput = maxInput;
    positionPIDWrappingMinInput = minInput;
  }

  public void disablePositionPIDWrapping() {
    pidController.setPositionPIDWrappingEnabled(false);

    positionPIDWrappingEnabled = false;
  }

  /**
   * Sets the current limit.
   *
   * @param limit The current limit the NEO will follow. The NEO will not pull more than this amount
   *     of current.
   */
  public void setCurrentLimit(int limit) {
    motor.setSmartCurrentLimit(limit);
  }

  public void setClosedLoopRampRate(double time) {
    motor.setClosedLoopRampRate(time);
  }

  public void setOpenLoopRampRate(double time) {
    motor.setOpenLoopRampRate(time);
  }

  /** Burns all configured settings to the NEO's flash memory. */
  public void burn() {
    REVLibError error = motor.burnFlash();
    for (int i = 0; i <= 4; i++) {
      error = motor.burnFlash();
      if (error == REVLibError.kOk) break;
    }
  }

  public void setPIDFeedbackDevice(SparkAbsoluteEncoder feedbackDevice) {
    pidController.setFeedbackDevice(feedbackDevice);
    remoteAbsoluteEncoder = feedbackDevice;
  }

  /**
   * Sets PID values.
   *
   * @param kP The proportional gain constant for PIDFF control.
   * @param kI The integral gain constant for PIDFF control.
   * @param kD The derivative gain constant for PIDFF control.
   * @param kIZ The I-Zone constant for PIDFF control. The I term will not start accumulating until
   *     the error is less than this value.
   * @param kFF The feedforward gain constant for PIDFF control.
   */
  public void configurePIDFF(double kP, double kI, double kD, double kIZ, double kFF, int slot) {
    pidController.setP(kP, slot);
    pidController.setI(kI, slot);
    pidController.setD(kD, slot);
    pidController.setIZone(kIZ, slot);
    pidController.setFF(kFF, slot);
  }

  public void configurePIDFF(double kP, double kI, double kD, double kIZ, double kFF) {
    configurePIDFF(kP, kI, kD, kIZ, kFF, 0);
  }

  public void configurePIDFF(double kP, double kI, double kD, double kFF) {
    configurePIDFF(kP, kI, kD, 0, kFF, 0);
  }

  public void configurePIDFF(double kP, double kI, double kD) {
    configurePIDFF(kP, kI, kD, 0, 0, 0);
  }

  public void setP(double kP) {
    setP(kP, 0);
  }

  public void setP(double kP, int slot) {
    configurePIDFF(kP, getI(), getD(), getIZ(), getFF(), slot);
  }

  public void setI(double kI) {
    setI(kI, 0);
  }

  public void setI(double kI, int slot) {
    configurePIDFF(getP(), kI, getD(), getIZ(), getFF(), slot);
  }

  public void setD(double kD) {
    setD(kD, 0);
  }

  public void setD(double kD, int slot) {
    configurePIDFF(getP(), getI(), kD, getIZ(), getFF(), slot);
  }

  public void setIZone(double kIZone) {
    setIZone(kIZone, 0);
  }

  public void setIZone(double kIZone, int slot) {
    configurePIDFF(getP(), getI(), getD(), kIZone, getFF(), slot);
  }

  public void setFF(double kFF) {
    setFF(kFF, 0);
  }

  public void setFF(double kFF, int slot) {
    configurePIDFF(getP(), getI(), getD(), getIZ(), kFF, slot);
  }

  public void setKS(double kS) {
    this.kS = kS;
  }

  public double getKS() {
    return this.kS;
  }

  /**
   * Gets the current applied % output from the motor.
   *
   * @return The current applied % output from the motor.
   */
  public double getOutput() {
    if (Robot.isReal()) {
      return motor.getAppliedOutput();
    } else {
      return motor.getAppliedOutput();
    }
  }

  /**
   * Clamps the applied power output for the NEO to this range.
   *
   * @param min Minimum power to output
   * @param max Maximum power to output.
   */
  public void setOutputRange(double min, double max) {
    pidController.setOutputRange(min, max);
  }

  /**
   * Gets the proportional gain constant for PIDFF controller.
   *
   * @return The proportional gain constant for PIDFF controller.
   */
  public double getP() {
    return pidController.getP();
  }

  /**
   * Gets the integral gain constant for PIDFF controller.
   *
   * @return The integral gain constant for PIDFF controller.
   */
  public double getI() {
    return pidController.getI();
  }

  /**
   * Gets the derivative gain constant for PIDFF controller.
   *
   * @return The derivative gain constant for PIDFF controller.
   */
  public double getD() {
    return pidController.getD();
  }

  /**
   * Gets the I-Zone constant for PIDFF controller.
   *
   * @return The I-Zone constant for PIDFF control.
   */
  public double getIZ() {
    return pidController.getIZone();
  }

  /**
   * Gets the feedforward gain constant for PIDFF controller.
   *
   * @return The feedforward gain constant for PIDFF controller.
   */
  public double getFF() {
    return pidController.getFF();
  }

  /**
   * Gets the CANID of the SparkMax running this motor.
   *
   * @return The CANID of the SparkMax running this motor.
   */
  public int getID() {
    return motor.getDeviceId();
  }

  /**
   * Gets the CANSparkMax object behind this motor.
   *
   * @return The CANSparkMax object behind this motor.
   */
  public CANSparkMax getMotor() {
    return motor;
  }

  public RelativeEncoder getEncoder() {
    return encoder;
  }

  public SparkAbsoluteEncoder getAbsoluteEncoder() {
    return motor.getAbsoluteEncoder(Type.kDutyCycle);
  }

  public SparkPIDController getPID() {
    return pidController;
  }

  // Documentation:
  // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
  public boolean changeStatusFrame(StatusFrame frame, int period) {
    REVLibError error = motor.setPeriodicFramePeriod(frame.getFrame(), period);

    return error == REVLibError.kOk;
  }

  public boolean resetStatusFrame(StatusFrame frame) {
    return changeStatusFrame(frame, frame.getDefaultPeriod());
  }

  // Documentation:
  // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
  public enum StatusFrame {
    APPLIED_FAULTS_FOLLOWER(PeriodicFrame.kStatus0, 10),
    VELO_TEMP_VOLTAGE_CURRENT(PeriodicFrame.kStatus1, 20),
    POSITION(PeriodicFrame.kStatus2, 20),
    ANALOG_VOLTAGE_VELO_POS(PeriodicFrame.kStatus3, 50),
    ALTERNATE_VELO_POS(PeriodicFrame.kStatus4, 20),
    ABSOLUTE_ENCODER_POS(PeriodicFrame.kStatus5, 200),
    ABSOLUTE_ENCODER_VELO(PeriodicFrame.kStatus6, 200);

    private final PeriodicFrame frame;
    private final int defaultPeriod; // ms

    StatusFrame(PeriodicFrame frame, int defaultPeriod) {
      this.frame = frame;
      this.defaultPeriod = defaultPeriod;
    }

    public PeriodicFrame getFrame() {
      return frame;
    }

    public int getDefaultPeriod() {
      return defaultPeriod;
    }
  }

  public enum ControlLoopType {
    POSITION,
    VELOCITY,
    PERCENT;
  }
}
