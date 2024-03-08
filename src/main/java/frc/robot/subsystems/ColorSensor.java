package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import java.util.ArrayList;

import com.revrobotics.ColorSensorV3;

public class ColorSensor extends SubsystemBase{
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final I2C.Port i2cPort2 = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor, m_colorSensor2;
  public int red, red2;
  //private ArrayList<Boolean> m_targetList;
  //private final int MAX_ENTRIES = 10;
  
  public ColorSensor() {
    m_colorSensor = new ColorSensorV3(i2cPort);
    m_colorSensor2 = new ColorSensorV3(i2cPort2);

    //m_targetList = new ArrayList<Boolean>(MAX_ENTRIES);
  }

  @Override
  public void periodic() {
    red = m_colorSensor.getRed();
    red2 = m_colorSensor2.getRed();

    SmartDashboard.putNumber("red", red);
    SmartDashboard.putNumber("red2", red2);

    //m_targetList.add(getSensor());
    
    //isDetected();

    /*if (m_targetList.size() >= MAX_ENTRIES) {
      m_targetList.remove(0);
    }*/

    if (getSensor()) {
      GlobalVariables.getInstance().extenderFull = true;
      SmartDashboard.putBoolean("Extender", true);
    } else {
      GlobalVariables.getInstance().extenderFull = false;
      SmartDashboard.putBoolean("Extender", false);
    }
  }

  public boolean getSensor() {
    if (red2 > 80 || red > 210) {
      return true;
    } else {
      return false;
    }
  }

  /*public void isDetected() {
    double positive = 0;
    for (Boolean target : m_targetList) {
      if (target) {
        positive += 1;
      }
    }

    if (positive > 0) {
      GlobalVariables.getInstance().extenderFull = true;
      SmartDashboard.putBoolean("Extender", true);
    } else {
      GlobalVariables.getInstance().extenderFull = false;
      SmartDashboard.putBoolean("Extender", false);
    }
  }*/
}