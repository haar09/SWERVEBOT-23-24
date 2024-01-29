package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class LimeLightFollowReflector extends Command{
    private final LimeLight m_LimeLight;
    private final SwerveSubsystem swerveSubsystem;
    private final int mode;

    public LimeLightFollowReflector(LimeLight subsystem, SwerveSubsystem swerveSubsystem, int mode){
        this.m_LimeLight = subsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.mode = mode;
        addRequirements(m_LimeLight, swerveSubsystem);
    }

    @Override
    public void initialize(){
    }

    PIDController forwardController = new PIDController(0, 0, 0);
    PIDController leftRighController = new PIDController(0.015, 0, 0);
    PIDController thetaController = new PIDController(PIDConstants.kPLimeLightRotate, 0, 0.0002);
    ChassisSpeeds chassisSpeeds, chassisSpeeds2;
    SwerveModuleState[] moduleStates;

    @Override
    public void execute(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);
        if (m_LimeLight.isTargetValid()) {
        if (mode==0) {
            Pose2d targetPose = m_LimeLight.getTargetPose();
            SmartDashboard.putNumber("x1", targetPose.getTranslation().getX());
            SmartDashboard.putNumber("y1", targetPose.getTranslation().getY());
            Pose2d offsetPose = new Pose2d(
                targetPose.getTranslation().getX(),
                targetPose.getTranslation().getY(), 
                targetPose.getRotation()
            );

            double forwardSpeed = forwardController.calculate(0, offsetPose.getTranslation().getX());
            double leftRightSpeed = leftRighController.calculate(0, offsetPose.getTranslation().getY());
            double angle = thetaController.calculate(0, -targetPose.getRotation().getDegrees());

            chassisSpeeds = new ChassisSpeeds(leftRightSpeed, forwardSpeed, angle);

        } else if (mode==1) {
            double tx = table.getEntry("tx").getDouble(0.0);
            if (tx!=0){
                double rightSpeed = leftRighController.calculate(0, tx);
                chassisSpeeds = new ChassisSpeeds(rightSpeed, 0, 0);
            }
        }
        }

        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }
        

    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
