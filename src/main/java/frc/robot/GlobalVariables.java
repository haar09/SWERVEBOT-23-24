package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.ShooterConstants;

public class GlobalVariables {
    private static GlobalVariables instance = new GlobalVariables();
    public double customRotateSpeed;
    public boolean extenderFull;
    public double speakerDistance;

    private GlobalVariables() {
        customRotateSpeed = 0;
        extenderFull = false;
        speakerDistance = 0;
    }

    public double speakerToAngle() {
                if (DriverStation.isAutonomous()) {
                    speakerDistance -= 0.26;
                }
                if (speakerDistance < 0.1) {
                    return (ShooterConstants.k0mAngle);   
                }
                else if (speakerDistance < 0.4) {
                    return (speakerDistance - 0.1) * ((ShooterConstants.k05mAngle-ShooterConstants.k0mAngle)/(0.3)) + ShooterConstants.k0mAngle;
                }
                else if (speakerDistance < 0.6) {
                    return (ShooterConstants.k05mAngle);
                }
                else if (speakerDistance < 0.9) {
                    return (speakerDistance - 0.6) * ((ShooterConstants.k1mAngle-ShooterConstants.k05mAngle)/(0.3)) + ShooterConstants.k05mAngle;
                }
                else if (speakerDistance < 1.1) {
                    return (ShooterConstants.k1mAngle);
                }
                else if (speakerDistance < 1.4) {
                    return (speakerDistance - 1.1) * ((ShooterConstants.k15mAngle-ShooterConstants.k1mAngle)/(0.3)) + ShooterConstants.k1mAngle;
                }
                else if (speakerDistance < 1.6) {
                    return (ShooterConstants.k15mAngle);
                }
                else if (speakerDistance < 1.9) {
                    return (speakerDistance - 1.6) * ((ShooterConstants.k2mAngle-ShooterConstants.k15mAngle)/(0.3)) + ShooterConstants.k15mAngle;
                }
                else if (speakerDistance < 2.1) {
                    return (ShooterConstants.k2mAngle);
                }
                else if (speakerDistance < 2.4) {
                    return (speakerDistance - 2.1) * ((ShooterConstants.k25mAngle-ShooterConstants.k2mAngle)/(0.3)) + ShooterConstants.k2mAngle;
                }
                else if (speakerDistance < 2.6) {
                    return (ShooterConstants.k25mAngle);
                }
                else {
                    return 0;
                }
    }

    public static GlobalVariables getInstance() {
        return instance;
    }
}