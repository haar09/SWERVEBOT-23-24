package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight extends SubsystemBase{
    private final NetworkTable m_limelightTable;
    private double tv, tx, ty, ta;
    private ArrayList<Double> m_targetList;
    private double[] target;
    private final int MAX_ENTRIES = 50;
    private final NetworkTableEntry m_isTargetValid;
    private final NetworkTableEntry m_ledEntry;
    private final HttpCamera limelightFeed;

    public LimeLight() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_targetList = new ArrayList<Double>(MAX_ENTRIES);

        m_isTargetValid = m_limelightTable.getEntry("isTargetValid");
        m_ledEntry = m_limelightTable.getEntry("ledMode");

        SmartDashboard.putNumber("Led Mode", m_ledEntry.getNumber(0).intValue());
        SmartDashboard.putBoolean("Limelight Target", m_isTargetValid.getBoolean(false));
        limelightFeed = new HttpCamera("stream", "http://10.69.89.11:5800/stream.mjpg",
                HttpCameraKind.kMJPGStreamer);
        // add limelightfeed to network tables
        Shuffleboard.getTab("stream").add(limelightFeed);
    }

    @Override
    public void periodic(){
        tv = m_limelightTable.getEntry("tv").getDouble(0);
        tx = m_limelightTable.getEntry("tx").getDouble(0);
        ty = m_limelightTable.getEntry("ty").getDouble(0);
        ta = m_limelightTable.getEntry("ta").getDouble(0);
        target = m_limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        m_isTargetValid.setBoolean(tv == 1.0);

        if (m_targetList.size() >= MAX_ENTRIES) {
            m_targetList.remove(0);
        }
        m_targetList.add(ta);
    }

    public Pose2d getTargetPose(){
        Pose2d pose = new Pose2d(target[0], target[1], new Rotation2d(Math.toRadians(target[5])));
        return pose;
    }

    public double getTX() {
        return tx;
    }

    public double getTY() {
        return ty;
    }
    
    public double getTA() {
        double sum = 0;
    
        for (Double num : m_targetList) { 		      
          sum += num.doubleValue();
        }
        return sum/m_targetList.size();
    }

    public boolean isTargetValid() {
        return (tv == 1.0); 
    }

    public boolean getLedMode(){
        int mode = m_ledEntry.getNumber(0).intValue();
        if (mode == 3){
            return true;
        } else {
            return false;
        }
    }

    public void setLedMode(boolean mode){
        int modeInt;
        if (mode){
            modeInt = 3;
        } else {
            modeInt = 1;
        }
        m_ledEntry.setNumber((modeInt));
    }

}
