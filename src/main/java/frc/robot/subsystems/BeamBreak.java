package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BeamBreak extends SubsystemBase{
  private final DigitalInput m_beamBreak;
  
  public BeamBreak() {
    m_beamBreak = new DigitalInput(0);
  }

  @Override
  public void periodic() {
      GlobalVariables.getInstance().extenderFull = !m_beamBreak.get();
      SmartDashboard.putBoolean("Extender", !m_beamBreak.get());
  }
}