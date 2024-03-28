package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import java.util.ArrayList;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight extends SubsystemBase{
    private double tx, ty, ta;
    private ArrayList<Double> m_targetList;
    private final int MAX_ENTRIES = 50;

    public LimeLight() {        
        m_targetList = new ArrayList<Double>(MAX_ENTRIES);

        SmartDashboard.putBoolean("Limelight Target", LimelightHelpers.getTV("limelight"));
    }

    @Override
    public void periodic(){
        if (LimelightHelpers.getTV("limelight")) {
            tx = LimelightHelpers.getTX("limelight");
            ty = LimelightHelpers.getTY("limelight");
            ta = LimelightHelpers.getTA("limelight");
            m_targetList.add(ta);
        }

        SmartDashboard.putBoolean("Limelight Target", LimelightHelpers.getTV("limelight"));

        if (m_targetList.size() >= MAX_ENTRIES) {
            m_targetList.remove(0);
        }
    }

    public LimelightHelpers.PoseEstimate getEstimatedGlobalPose() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }

    public Translation3d cameraToTarget() {
        return LimelightHelpers.getTargetPose3d_CameraSpace("limelight").getTranslation();
    }

    public double getTargetID() {
        return LimelightHelpers.getFiducialID("limelight");
    }

    public double getTXwithOffset() {
        Math.atan(DriveConstants.kSagSolArasi/2 / Math.hypot(cameraToTarget().getX(), cameraToTarget().getY()));

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
        return LimelightHelpers.getTV("limelight"); 
    }
}
