package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.VisionConstants;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class OV9281 extends SubsystemBase{
    private final PhotonCamera camera1, camera2;
    private PhotonPipelineResult result1, result2;
    private final PhotonPoseEstimator photonEstimator1, photonEstimator2;
    private double lastEstTimestamp = 0;

    public OV9281() {
        camera1 = new PhotonCamera("Arducam_OV9281_USB_Camera_001");
        camera2 = new PhotonCamera("Arducam_OV9281_USB_Camera_002");

        result1 = camera1.getLatestResult();
        result2 = camera2.getLatestResult();

        photonEstimator1 =
                new PhotonPoseEstimator(
                        VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1, VisionConstants.kRobotToCam1);
        photonEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonEstimator2 =
                new PhotonPoseEstimator(
                        VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera2, VisionConstants.kRobotToCam2);
        photonEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        GlobalVariables.getInstance().isDetected(isTargetValid1(), 1);
        GlobalVariables.getInstance().isDetected(isTargetValid2(), 2);
    }

    @Override
    public void periodic(){
        result1 = camera1.getLatestResult();
        result2 = camera2.getLatestResult();

        GlobalVariables.getInstance().isDetected(isTargetValid1(), 1);
        GlobalVariables.getInstance().isDetected(isTargetValid2(), 2);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose1() {
        var visionEst = photonEstimator1.update();
        double latestTimestamp = camera1.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose2() {
        var visionEst = photonEstimator2.update();
        double latestTimestamp = camera2.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public Matrix<N3, N1> getEstimationStdDevs1(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        var targets = result1.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator1.getFieldTags().getTagPose(tgt.getFiducialId());
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

    public Matrix<N3, N1> getEstimationStdDevs2(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        var targets = result2.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator2.getFieldTags().getTagPose(tgt.getFiducialId());
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

    public boolean isTargetValid1() {
        return result1.hasTargets(); 
    }

    public boolean isTargetValid2() {
        return result2.hasTargets(); 
    }
}