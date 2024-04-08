package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
    private final CANcoder absoluteEncoder;

    private final PIDController turningPidController;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, double absoluteEncoderOffset){

            driveMotor = new CANSparkMax(driveMotorId, CANSparkMax.MotorType.kBrushless);
            turningMotor = new CANSparkMax(turningMotorId, CANSparkMax.MotorType.kBrushless);

            driveMotor.setInverted(driveMotorReversed);
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            driveMotor.setSmartCurrentLimit(30);
            turningMotor.setInverted(turningMotorReversed);
            turningMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
            turningMotor.setSmartCurrentLimit(20);

            driveEncoder = driveMotor.getEncoder();
            turningEncoder = turningMotor.getEncoder();

            absoluteEncoder = new CANcoder(absoluteEncoderId);
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
            config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1; //plus minus or unsigned
            config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            absoluteEncoder.getConfigurator().apply(config);

            driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
            driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
            turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
            turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

            turningPidController = new PIDController(PIDConstants.kPTurning, 0, PIDConstants.kDTurning);
            turningPidController.enableContinuousInput(-Math.PI, Math.PI);

            driveMotor.burnFlash();
            turningMotor.burnFlash();
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

    public double getAbsolutePosition(){
        return (absoluteEncoder.getPosition().getValueAsDouble() * 360);
    }

    public double getAbsoluteEncoderRad() {
		double angle = Math.toRadians(absoluteEncoder.getPosition().getValueAsDouble() * 360);
        return angle;
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
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond
        * Math.cos(Math.abs(getTurningPosition()-state.angle.getRadians()))
        );
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

    public double getAmperage(){
        return turningMotor.getOutputCurrent();
    }
}
