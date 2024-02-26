package frc.robot;

public class GlobalVariables {
    private static GlobalVariables instance = new GlobalVariables();
    public double rotateToTargetSpeed;
    public boolean extenderFull;

    private GlobalVariables() {
        rotateToTargetSpeed = 0;
        extenderFull = false;
    }

    public static GlobalVariables getInstance() {
        return instance;
    }
}