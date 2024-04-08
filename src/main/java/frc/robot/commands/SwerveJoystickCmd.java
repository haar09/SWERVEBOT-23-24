package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command{

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunc, ySpdFunc, turnSpdFunc;
    private final Supplier<Boolean> fieldOrientedFunc, slowMode, boost;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunc, Supplier<Double> ySpdFunc, Supplier<Double> turnSpdFunc,
            Supplier<Boolean> fieldOrientedFunc, Supplier<Boolean> slowMode, Supplier<Boolean> boost){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunc = xSpdFunc;
        this.ySpdFunc = ySpdFunc;
        this.turnSpdFunc = turnSpdFunc;
        this.fieldOrientedFunc = fieldOrientedFunc;
        this.slowMode = slowMode;
        this.boost = boost;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        // get joystick
        double xSpeed = xSpdFunc.get();
        double ySpeed = ySpdFunc.get();
        double turningSpeed = turnSpdFunc.get();

        // deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0;

        // rate limit for smooth acceleration
        if (boost.get()) {
                xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveBoostSpeedMetersPerSecond;
                ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveBoostSpeedMetersPerSecond;
        } else {
                xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
                ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        }
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        if (!slowMode.get()){
            xSpeed *= DriveConstants.kTeleDriveSlowModeMultiplier;
            ySpeed *= DriveConstants.kTeleDriveSlowModeMultiplier;
            turningSpeed *= DriveConstants.kTeleDriveSlowModeMultiplier;
        }

        if (GlobalVariables.getInstance().customRotateSpeed != 0){
            turningSpeed = GlobalVariables.getInstance().customRotateSpeed;
        }
        
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunc.get()) {
            //field oriented
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            //robot oriented
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
