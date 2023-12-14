package frc.robot;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleboardInfo {
    private final ShuffleboardTab m_driver_tab;

    // add values here
    private final GenericEntry m_isTargetValid;

    private static ShuffleboardInfo instance = null;

    private ShuffleboardInfo(){
        m_driver_tab = Shuffleboard.getTab("Driver");

        m_driver_tab.add("Limelight", new HttpCamera("Limelight", "http://10.69.89.11:5800/stream.mjpg"))
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(4, 0)
            .withSize(3, 3);
    
        m_isTargetValid = m_driver_tab.add("Valid Target?", false)
            .withPosition(2, 0)
            .withSize(1, 1)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();
    }

    public static ShuffleboardInfo getInstance(){
        if( instance == null ){
            instance = new ShuffleboardInfo();
        }

        return instance;
    }

    public GenericEntry getTargetEntry() {
        return m_isTargetValid;
    }
}
