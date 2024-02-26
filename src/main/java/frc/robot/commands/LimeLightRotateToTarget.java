package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class LimeLightRotateToTarget extends Command{
    private final LimeLight m_LimeLight;
    private final SwerveSubsystem swerveSubsystem;
    private double tx;
    
    public LimeLightRotateToTarget(LimeLight subsystem, SwerveSubsystem swerveSubsystem){
        this.m_LimeLight = subsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(m_LimeLight);
    }

    PIDController thetaController = new PIDController(PIDConstants.kPLimeLightRotate, 0, 0.000018);
    @Override
    public void initialize(){
        System.out.println("START");
    }

    @Override
    public void execute(){
        tx = -m_LimeLight.getTX();
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, thetaController.calculate(0, tx)));

        swerveSubsystem.setModuleStates(moduleStates);
    }

    public void end(boolean interrupted){
        GlobalVariables.getInstance().rotateToTargetSpeed = 0;
        thetaController.close();
    }

    @Override
    public boolean isFinished(){
        return Math.abs(tx) < VisionConstants.kTXTolerance;
    }
}
