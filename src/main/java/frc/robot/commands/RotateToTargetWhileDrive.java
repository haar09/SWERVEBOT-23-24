package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateToTargetWhileDrive extends Command{
    
    private SwerveSubsystem swerveSubsystem;
    private double angle;
    private Translation2d robotToTarget;
    private Pose2d kSpeakerApriltagPose;

    public RotateToTargetWhileDrive(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
    }

    PIDController thetaController = new PIDController(PIDConstants.kPLimeLightRotate, 0, PIDConstants.kDLimeLightRotate);
    @Override
    public void initialize(){
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            kSpeakerApriltagPose = VisionConstants.kTagLayout.getTagPose(4).get().toPose2d();
        } else {
            kSpeakerApriltagPose = VisionConstants.kTagLayout.getTagPose(8).get().toPose2d();
        }
    }

    @Override
    public void execute(){
        robotToTarget = kSpeakerApriltagPose.minus(swerveSubsystem.getPose()).getTranslation();

        if (DriverStation.getAlliance().get() == Alliance.Red) {
        angle = 
        Math.toDegrees(Math.atan2(
            robotToTarget.getY(),
            robotToTarget.getX()
        ));
        } else {
        angle = 
        Math.toDegrees(Math.atan2(
            robotToTarget.getY(),
            -robotToTarget.getX()
        ));
        }

        angle = (angle + 180) % 360 - 180;

        thetaController.setSetpoint(angle);

        GlobalVariables.getInstance().customRotateSpeed = thetaController.calculate(swerveSubsystem.getPose().getRotation().getDegrees());
    }

    public void end(boolean interrupted){
        GlobalVariables.getInstance().customRotateSpeed = 0;
        thetaController.close();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
