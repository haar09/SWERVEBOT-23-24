package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShuffleboardInfo;

import java.util.ArrayList;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase{
    private final NetworkTable m_limelightTable;
    private double tv, tx, ty, ta;
    private ArrayList<Double> m_targetList;
    private final int MAX_ENTRIES = 50;
    private final GenericEntry m_isTargetValid;
    private final NetworkTableEntry m_ledEntry;

    public LimeLight(){
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_targetList = new ArrayList<Double>(MAX_ENTRIES);
        m_isTargetValid = ShuffleboardInfo.getInstance().getTargetEntry();
        m_ledEntry = m_limelightTable.getEntry("ledMode");
    }   

    @Override
    public void periodic(){
        tv = m_limelightTable.getEntry("tv").getDouble(0);
        tx = m_limelightTable.getEntry("tx").getDouble(0);
        ty = m_limelightTable.getEntry("ty").getDouble(0);
        ta = m_limelightTable.getEntry("ta").getDouble(0);

        m_isTargetValid.setBoolean(tv == 1.0);

        if (m_targetList.size() >= MAX_ENTRIES) {
            m_targetList.remove(0);
        }
        m_targetList.add(ta);
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
