package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule{
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;
    private final PIDController velocityPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
            
            this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
            this.absoluteEncoderReversed = absoluteEncoderReversed;

            driveMotor = new CANSparkMax(driveMotorId, CANSparkMax.MotorType.kBrushless);
            turningMotor = new CANSparkMax(turningMotorId, CANSparkMax.MotorType.kBrushless);

            driveMotor.setInverted(driveMotorReversed);
            turningMotor.setInverted(turningMotorReversed);

            driveEncoder = driveMotor.getEncoder();
            turningEncoder = turningMotor.getEncoder();

            driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
            driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
            turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
            turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

            velocityPidController = new PIDController(PIDConstants.kPVelocity, 0, PIDConstants.kDVelocity);
            velocityPidController.enableContinuousInput(-DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);

            turningPidController = new PIDController(PIDConstants.kPTurning, 0, 0);
            turningPidController.enableContinuousInput(-Math.PI, Math.PI);

            resetEncoders();
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = turningEncoder.getPosition();
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        double currentVelocity = getDriveVelocity();
        double velocityError = state.speedMetersPerSecond - currentVelocity;

        double output = velocityPidController.calculate(velocityError, currentVelocity);

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(output);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

        SmartDashboard.putString("SwerveDrive[" + driveMotor.getDeviceId() + "] VELOCITY", Double.toString(getDriveVelocity()));
        SmartDashboard.putString("SwerveDrive[" + driveMotor.getDeviceId() + "] POSITION", Double.toString(getDrivePosition()));

        SmartDashboard.putString("SwerveTurn[" + turningMotor.getDeviceId() + "] VELOCITY", Double.toString(getTurningVelocity()));
        SmartDashboard.putString("SwerveTurn[" + turningMotor.getDeviceId() + "] POSITION", Double.toString(getTurningPosition()));
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
