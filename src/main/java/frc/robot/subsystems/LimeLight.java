package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;

public class LimeLight extends SubsystemBase{
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    public static class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds;

        public PoseEstimate(Pose2d pose, double timestampSeconds) {

            this.pose = pose;
            this.timestampSeconds = timestampSeconds;        }
    }

    public LimeLight() {        
        GlobalVariables.getInstance().isDetected(isTargetValid(), 0);
    }

    @Override
    public void periodic(){
        GlobalVariables.getInstance().isDetected(isTargetValid(), 0);
    }

    public PoseEstimate getEstimatedGlobalPose() {
        var poseEntry = limelightTable.getEntry("botpose_wpiblue");

        var botposeArray = poseEntry.getDoubleArray(new double[0]);
        
        try {
        var timestamp = (poseEntry.getLastChange() / 1000000.0) - (botposeArray[6]/1000.0);

        return new PoseEstimate(
            new Pose2d(
                new Translation2d(botposeArray[0], botposeArray[1]),
                new Rotation2d(Math.toRadians(botposeArray[5]))
            ),
            timestamp);
        } catch (Exception e) {
            return new PoseEstimate(
            new Pose2d(
                new Translation2d(0,0),
                new Rotation2d(Math.toRadians(0))
            ),
            0);
        }
    }

    public boolean isTargetValid() {
        return limelightTable.getEntry("tv").getDouble(0) == 1; 
    }
}
