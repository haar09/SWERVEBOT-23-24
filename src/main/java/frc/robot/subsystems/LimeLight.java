package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

public class LimeLight extends SubsystemBase{
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private final PhotonPoseEstimator photonEstimator;
    private double tx, ty, ta;
    private ArrayList<Double> m_targetList;
    private final int MAX_ENTRIES = 50;
    private double lastEstTimestamp = 0;

    public LimeLight() {
        camera = new PhotonCamera("Camera_Module_v1");
        result = camera.getLatestResult();

        photonEstimator =
                new PhotonPoseEstimator(
                        VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        
        m_targetList = new ArrayList<Double>(MAX_ENTRIES);

        SmartDashboard.putBoolean("Limelight Target", result.hasTargets());
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
        }

        SmartDashboard.putBoolean("Limelight Target", result.hasTargets());

        if (m_targetList.size() >= MAX_ENTRIES) {
            m_targetList.remove(0);
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        var targets = result.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public Transform3d cameraToTarget() {
        return target.getBestCameraToTarget();
    }

    public double getTargetID() {
        return target.getFiducialId();
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
        return result.hasTargets(); 
    }

    public List<PhotonTrackedTarget> getTargets() {
        return result.getTargets();
    }
}
