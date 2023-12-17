package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class LimeLightFollowReflector extends CommandBase{
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
        m_LimeLight.setLedMode(true);
    }

    PIDController forwardController = new PIDController(0.01, 0, 0);
    PIDController leftRighController = new PIDController(0.01, 0, 0);
    ChassisSpeeds chassisSpeeds;
    SwerveModuleState[] moduleStates;

    @Override
    public void execute(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        if (mode==0){
            double angleToGoalDegrees = OIConstants.kLimeLightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    
            double distanceFromLimelightToGoalMeters = (OIConstants.kGoalHeightMeters - OIConstants.kLimeLightHeightMeters) / Math.tan(angleToGoalRadians);
        
            LimeLightRotateToTarget limeLightRotateToTarget = new LimeLightRotateToTarget(m_LimeLight);
            limeLightRotateToTarget.execute();
            chassisSpeeds = new ChassisSpeeds(0, 0, GlobalVariables.getInstance().rotateToTargetSpeed);
            moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);   
    
            double forwardSpeed = forwardController.calculate(0, distanceFromLimelightToGoalMeters);
            System.out.println("Forward Speed: " + forwardSpeed);
            chassisSpeeds = new ChassisSpeeds(forwardSpeed, 0, 0);
        } else {
            double tx = -table.getEntry("tx").getDouble(0.0);
            if (mode==1 && tx>0){
                double rightSpeed = leftRighController.calculate(0, tx);
                System.out.println("Right Speed: " + rightSpeed);
                chassisSpeeds = new ChassisSpeeds(0, rightSpeed, 0);
            } else if (mode==2 && tx<0) {
                double leftSpeed = leftRighController.calculate(0, tx);
                System.out.println("Left Speed: " + leftSpeed);
                chassisSpeeds = new ChassisSpeeds(0, leftSpeed, 0);
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
