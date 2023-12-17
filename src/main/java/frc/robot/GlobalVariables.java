package frc.robot;

public class GlobalVariables {
    private static GlobalVariables instance = new GlobalVariables();
    public double rotateToTargetSpeed;

    private GlobalVariables() {
        rotateToTargetSpeed = 0;
    }

    public static GlobalVariables getInstance() {
        return instance;
    }
}