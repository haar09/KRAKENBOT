// Developed by Reza from Team Spyder 1622

package frc.robot.util;

import com.revrobotics.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class NEO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  public final SparkMaxConfig config = new SparkMaxConfig();

  private ControlLoopType controlType = ControlLoopType.PERCENT;
  private double targetPosition = 0;
  private double targetVelocity = 0;

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
    motor = new SparkMax(id, MotorType.kBrushless);

    Timer.delay(0.050);

    // If a parameter set fails, this will add more time to alleviate any bus
    // traffic default is 20ms
    motor.setCANTimeout(250);

    config.inverted(reversed).idleMode(mode).smartCurrentLimit(currentLimit);

    encoder = motor.getEncoder();
    
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

  public IdleMode getIdleMode() {
    return motor.configAccessor.getIdleMode();
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void set(double percent) {
    setVoltage(percent * RobotController.getBatteryVoltage());
    controlType = ControlLoopType.PERCENT;
  }

  /**
   * Gets the position of the NEO in rotations.
   *
   * @return The position of the NEO in rotations relative to the last 0 position.
   */
  public double getPosition() {
    double pos = encoder.getPosition();

    if (Robot.isSimulation() && controlType == ControlLoopType.VELOCITY) {
      pos /= motor.configAccessor.encoder.getVelocityConversionFactor();
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
    config.inverted(inverted);
    burn();
  }

  /** Burns all configured settings to the NEO's flash memory. */
  public void burn() {
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
   * Gets the CANID of the SparkMax running this motor.
   *
   * @return The CANID of the SparkMax running this motor.
   */
  public int getID() {
    return motor.getDeviceId();
  }

  /**
   * Gets the SparkMax object behind this motor.
   *
   * @return The SparkMax object behind this motor.
   */
  public SparkMax getMotor() {
    return motor;
  }

  public RelativeEncoder getEncoder() {
    return encoder;
  }

  public enum ControlLoopType {
    POSITION,
    VELOCITY,
    PERCENT;
  }
}
