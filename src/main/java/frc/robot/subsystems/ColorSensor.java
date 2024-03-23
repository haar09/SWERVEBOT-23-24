package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

import com.revrobotics.ColorSensorV3;

public class ColorSensor extends SubsystemBase{
  private final I2C.Port i2cPort = I2C.Port.kOnboard;  
  private final ColorSensorV3 m_colorSensor;
  public int red;
  private ArrayList<Boolean> m_targetList;
  private final int MAX_ENTRIES = 60;
  
  public ColorSensor() {
    m_colorSensor = new ColorSensorV3(i2cPort);

    m_targetList = new ArrayList<Boolean>(MAX_ENTRIES);
    SmartDashboard.putNumber("COLOR SENSOR", 265);
  }

  @Override
  public void periodic() {
    red = m_colorSensor.getRed();

    SmartDashboard.putNumber("red", red);
    SmartDashboard.putNumber("proximity", m_colorSensor.getProximity());

    m_targetList.add(getSensor());
    
    isDetected();

    if (m_targetList.size() >= MAX_ENTRIES) {
      m_targetList.remove(0);
    }
  }

  public boolean getSensor() {
    if (red > SmartDashboard.getNumber("COLOR SENSOR", 265)) {
      return true;
    } else {
      return false;
    }
  }

  public void isDetected() {
    double positive = 0;
    for (Boolean target : m_targetList) {
      if (target) {
        positive += 1;
      }
    }

    if (positive > 1) {
      GlobalVariables.getInstance().extenderFull = true;
      SmartDashboard.putBoolean("Extender", true);
    } else {
      GlobalVariables.getInstance().extenderFull = false;
      SmartDashboard.putBoolean("Extender", false);
    }
  }
}