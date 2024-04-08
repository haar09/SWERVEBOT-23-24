package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase{
    private final PhotonCamera camera;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private double tx, ty, ta;
    private ArrayList<Double> m_targetList;
    private final int MAX_ENTRIES = 50;

    public ObjectDetection() {        
        camera = new PhotonCamera("Logitech");

        result = camera.getLatestResult();

        m_targetList = new ArrayList<Double>(MAX_ENTRIES);
    }

    @Override
    public void periodic(){
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            tx = target.getYaw();
            ty = target.getPitch();
            ta = target.getArea();
            m_targetList.add(ta);
        } else {
            tx = 0;
            ty = 0;
        }

        if (m_targetList.size() >= MAX_ENTRIES) {
            m_targetList.remove(0);
        }
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
        return result.hasTargets(); 
    }
}
