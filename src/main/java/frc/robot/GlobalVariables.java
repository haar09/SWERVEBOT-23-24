package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

public class GlobalVariables {
    private static GlobalVariables instance = new GlobalVariables();
    public double customRotateSpeed;
    public boolean extenderFull;
    public double speakerDistance;
    public DriverStation.Alliance alliance;
    public boolean[] detectedList = {false, false, false};

    private GlobalVariables() {
        customRotateSpeed = 0;
        extenderFull = false;
        speakerDistance = 0;
        alliance = null;
    }

    public void isDetected(boolean detected, int index) {
        detectedList[index] = detected;
        if (detectedList[0] || detectedList[1] || detectedList[2]) {
            SmartDashboard.putBoolean("Limelight Target", true);
        } else {
            SmartDashboard.putBoolean("Limelight Target", false);
        }
    }

    public double speakerToAngle() {
        if (speakerDistance < 1) {
            return (VisionConstants.y_ArmAngle[0]);   
        } else if (speakerDistance < 3.5) {
            return VisionConstants.angleFunction.value(speakerDistance);
        }
        else {
            return 0;
        }
    }

    public static GlobalVariables getInstance() {
        return instance;
    }
}