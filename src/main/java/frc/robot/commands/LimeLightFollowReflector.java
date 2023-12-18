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

    PIDController forwardController = new PIDController(3, 0, 0);
    PIDController leftRighController = new PIDController(0.3, 0, 0);
    ChassisSpeeds chassisSpeeds, chassisSpeeds2;
    SwerveModuleState[] moduleStates;

    @Override
    public void execute(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);
        if (mode==0){
            double angleToGoalDegrees = OIConstants.kLimeLightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    
            double distanceFromLimelightToGoalMeters = (OIConstants.kGoalHeightMeters - OIConstants.kLimeLightHeightMeters) / Math.tan(angleToGoalRadians);
            if (targetOffsetAngle_Vertical==0) {distanceFromLimelightToGoalMeters = 0;}
            LimeLightRotateToTarget limeLightRotateToTarget = new LimeLightRotateToTarget(m_LimeLight);
            limeLightRotateToTarget.execute();
    
            double forwardSpeed = forwardController.calculate(0, distanceFromLimelightToGoalMeters-1);

            double align = GlobalVariables.getInstance().rotateToTargetSpeed/2;

            align = Math.abs(align) > 0.07 ? align : 0;

            chassisSpeeds = new ChassisSpeeds(0, -forwardSpeed, align);
        } else {
            double tx = -table.getEntry("tx").getDouble(0.0);
            if (mode==1 && tx>0){
                double rightSpeed = leftRighController.calculate(0, tx);
                chassisSpeeds = new ChassisSpeeds(rightSpeed, 0, 0);
            } else if (tx<0) {
                double leftSpeed = leftRighController.calculate(0, tx);
                chassisSpeeds = new ChassisSpeeds(leftSpeed, 0, 0);
            } else{
            System.out.println("NONE");
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
