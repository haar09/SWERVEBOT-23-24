package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule{
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    private final CANCoder absoluteEncoder;

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
            
            this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
            this.absoluteEncoderReversed = absoluteEncoderReversed;

            driveMotor = new CANSparkMax(driveMotorId, CANSparkMax.MotorType.kBrushless);
            turningMotor = new CANSparkMax(turningMotorId, CANSparkMax.MotorType.kBrushless);

            driveMotor.setInverted(driveMotorReversed);
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            turningMotor.setInverted(turningMotorReversed);
            turningMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

            driveEncoder = driveMotor.getEncoder();
            turningEncoder = turningMotor.getEncoder();

            absoluteEncoder = new CANCoder(absoluteEncoderId);
            absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

            driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
            driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
            turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
            turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

            turningPidController = new PIDController(PIDConstants.kPTurning, 0, 0);
            turningPidController.enableContinuousInput(-Math.PI, Math.PI);

            resetEncoders();
    }

    public void switchIdleMode(){
        if (driveMotor.getIdleMode() == CANSparkMax.IdleMode.kCoast) {
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        } else {
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }
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
        double angle = absoluteEncoder.getAbsolutePosition() * ModuleConstants.kTurningEncoderRot2Rad;
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
        
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivePosition(),
            new Rotation2d(getTurningPosition())
        );
    }
    
    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
