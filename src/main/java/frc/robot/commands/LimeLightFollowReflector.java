package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class LimeLightFollowReflector extends Command{
    private final LimeLight m_LimeLight;
    private final SwerveSubsystem swerveSubsystem;

    public LimeLightFollowReflector(LimeLight subsystem, SwerveSubsystem swerveSubsystem){
        this.m_LimeLight = subsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(m_LimeLight, swerveSubsystem);
    }

    @Override
    public void initialize(){
    }

    PIDController forwardController = new PIDController(3, 0, 0);
    PIDController leftRighController = new PIDController(1, 0, 0);
    PIDController thetaController = new PIDController(PIDConstants.kPLimeLightRotate, 0, 0.000018);
    ChassisSpeeds chassisSpeeds, chassisSpeeds2;
    SwerveModuleState[] moduleStates;

    @Override
    public void execute(){
        if (m_LimeLight.isTargetValid()) {
            double[] targetPose = m_LimeLight.getTargetPose();
            SmartDashboard.putNumber("x1", targetPose[0]);
            SmartDashboard.putNumber("y1", targetPose[1]);

            double forwardSpeed = forwardController.calculate(0, -targetPose[1]);
            double leftRightSpeed = leftRighController.calculate(0, targetPose[0]);
            //double angle = thetaController.calculate(0, -targetPose.getRotation().getDegrees());

            chassisSpeeds = new ChassisSpeeds(leftRightSpeed, forwardSpeed, 0);
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
