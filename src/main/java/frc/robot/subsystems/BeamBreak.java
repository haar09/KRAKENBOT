package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BeamBreak extends SubsystemBase{
  private final DigitalInput m_beamBreak, lower_beambreak;
  public boolean upper_value = false;
  public boolean lower_value = false;

  public BeamBreak() {
    m_beamBreak = new DigitalInput(0);
    lower_beambreak = new DigitalInput(2);
  }

  @Override
  public void periodic() {
      upper_value = !m_beamBreak.get();
      GlobalVariables.getInstance().extenderFull = upper_value;
      Logger.recordOutput("Beam Break/Upper Beam", upper_value);
      SmartDashboard.putBoolean("Extender", upper_value);

      lower_value = !lower_beambreak.get();
      Logger.recordOutput("Beam Break/Lower Beam", lower_value);
  }
}