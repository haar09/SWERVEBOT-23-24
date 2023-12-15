package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class LimeLightRotateToTarget extends CommandBase{
    private final LimeLight m_LimeLight;
    private final SwerveSubsystem swerveSubsystem;
    public LimeLightRotateToTarget(LimeLight subsystem, SwerveSubsystem swerveSubsystem){
        this.m_LimeLight = subsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(m_LimeLight, swerveSubsystem);
    }

    @Override
    public void initialize(){
        m_LimeLight.setLedMode(true);
    }

    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

    @Override
    public void execute(){
        double tx = m_LimeLight.getTX();

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        double turningSpeed = thetaController.calculate(0, tx);

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(0, 0, turningSpeed);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

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
