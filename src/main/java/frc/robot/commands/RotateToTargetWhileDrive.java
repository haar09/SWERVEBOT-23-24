package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateToTargetWhileDrive extends Command{
        
    private SwerveSubsystem swerveSubsystem;
    private double angle;
    private Translation2d robotToTarget, kSpeakerApriltagPose;
    private XboxController operatorController;
    
     PIDController thetaController = new PIDController(PIDConstants.kPLimeLightRotate, 0, PIDConstants.kDLimeLightRotate);

    public RotateToTargetWhileDrive(SwerveSubsystem swerveSubsystem, XboxController operatorController){
        this.swerveSubsystem = swerveSubsystem;
        this.operatorController = operatorController;
        thetaController.enableContinuousInput(0, 360);
    }
    
    @Override
    public void initialize(){
        if (GlobalVariables.getInstance().alliance == Alliance.Red) {
            kSpeakerApriltagPose = VisionConstants.kTagLayout.getTagPose(4).get().getTranslation().toTranslation2d();
        } else {
            kSpeakerApriltagPose = VisionConstants.kTagLayout.getTagPose(7).get().getTranslation().toTranslation2d();
        }
    }

    @Override
    public void execute(){
        robotToTarget = kSpeakerApriltagPose.minus(swerveSubsystem.getPose().getTranslation());

        angle = robotToTarget.getAngle().getDegrees();

        thetaController.setSetpoint(angle);

        GlobalVariables.getInstance().customRotateSpeed = thetaController.calculate(swerveSubsystem.getPose().getRotation().getDegrees());

        System.out.println(thetaController.getPositionError());
        if (Math.abs(thetaController.getPositionError()) < 6){
            operatorController.setRumble(RumbleType.kBothRumble, 0.15);
        }         
    }

    public void end(boolean interrupted){
        operatorController.setRumble(RumbleType.kBothRumble, 0);
        GlobalVariables.getInstance().customRotateSpeed = 0;
        thetaController.close();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}