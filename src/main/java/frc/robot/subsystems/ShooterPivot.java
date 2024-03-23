package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivot extends SubsystemBase{
    
    private final CANSparkMax pivotMotor;

    private final CANcoder absoluteEncoder;

    private final PIDController anglePidController;

    public ShooterPivot() {
        pivotMotor = new CANSparkMax(ShooterConstants.kPivotMotorId, CANSparkMax.MotorType.kBrushless);
        absoluteEncoder = new CANcoder(ShooterConstants.kAbsoluteEncoderId);

        pivotMotor.setInverted(ShooterConstants.kPivotMotorReversed);
        pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = ShooterConstants.kAbsoluteEncoderOffset;
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(config);

        pivotMotor.burnFlash();

        anglePidController = new PIDController(ShooterConstants.kAngleP, ShooterConstants.kAngleI, ShooterConstants.kAngleD);
        anglePidController.setTolerance(ShooterConstants.kAngleToleranceRad);
        anglePidController.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putNumber("customAngle", 30);

        resetEncoders();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Angle", Math.toDegrees(getAngle()));
    }

    public double getAngle(){
        return absoluteEncoder.getPosition().getValueAsDouble() * ShooterConstants.kPivotMotorRot2Rad;
    }

    public double getAbsolutePosition(){
        return (absoluteEncoder.getPosition().getValueAsDouble() * 360);
    }

    public void resetEncoders(){
        absoluteEncoder.setPosition(getAbsolutePosition() / 360);
    }

    public void setDesiredAngle(double angle){
        angle = Math.toRadians(angle);
        if (Math.abs(angle) < ShooterConstants.kAngleToleranceRad) {
            stop();
            return;
        }

        if (angle > ShooterConstants.kMaxShooterAngle) {
            angle = ShooterConstants.kMaxShooterAngle;
        } else if (angle < ShooterConstants.kMinShooterAngle) {
            angle = ShooterConstants.kMinShooterAngle;
        }

        pivotMotor.set(anglePidController.calculate(getAngle(), angle));
    }
    
    public void stop(){
        pivotMotor.set(0);
    }
}