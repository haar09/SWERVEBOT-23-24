package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.GlobalVariables;

public class Shooter extends SubsystemBase{

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    /*private final SparkPIDController leftPidController;
    private final SparkPIDController rightPidController;*/

    private final LEDSubsystem ledSubsystem;

    public boolean ledIdle;

    public Shooter(LEDSubsystem m_LedSubsystem) {
        ledSubsystem = m_LedSubsystem;

        leftMotor = new CANSparkMax(ShooterConstants.kShooterMotorLeftId, CANSparkMax.MotorType.kBrushless);
        rightMotor = new CANSparkMax(ShooterConstants.kShooterMotorRightId, CANSparkMax.MotorType.kBrushless);
        
        /*leftPidController = leftMotor.getPIDController();
        rightPidController = rightMotor.getPIDController();

        leftMotor.enableVoltageCompensation(10.5);
        leftMotor.setSmartCurrentLimit(40);
        leftPidController.setP(ShooterConstants.kShooterMotorP);
        leftPidController.setI(ShooterConstants.kShooterMotorI);
        leftPidController.setD(ShooterConstants.kShooterMotorD);
        leftPidController.setFF(ShooterConstants.kShooterMotorFF);

        rightMotor.enableVoltageCompensation(10.5);
        rightMotor.setSmartCurrentLimit(40);
        rightPidController.setP(ShooterConstants.kShooterMotorP);
        rightPidController.setI(ShooterConstants.kShooterMotorI);
        rightPidController.setD(ShooterConstants.kShooterMotorD);
        rightPidController.setFF(ShooterConstants.kShooterMotorFF);*/

        leftMotor.setInverted(ShooterConstants.kShooterMotorLeftReversed);
        rightMotor.setInverted(ShooterConstants.kShooterMotorRightReversed);

        leftMotor.burnFlash();
        rightMotor.burnFlash();

        ledIdle = true;
    }

    public void setSpeakerSpeed() {
        /*while (leftMotor.getAppliedOutput() < ShooterConstants.kShooterMotorLeftSpeed-0.07 && rightMotor.getAppliedOutput() < ShooterConstants.kShooterMotorRightSpeed-0.07) {
            leftPidController.setReference(ShooterConstants.kShooterMotorLeftSpeed, CANSparkMax.ControlType.kVelocity);
            rightPidController.setReference(ShooterConstants.kShooterMotorRightSpeed, CANSparkMax.ControlType.kVelocity);
            ledIdle = false;
            ledSubsystem.rainbowMode = false;
            ledSubsystem.setColor(0, 0, 255);
        }*/
        leftMotor.set(ShooterConstants.kSpeakerSpeedLeft);
        rightMotor.set(ShooterConstants.kSpeakerSpeedRight);
        while (leftMotor.getAppliedOutput() < ShooterConstants.kSpeakerSpeedLeft-0.05 && rightMotor.getAppliedOutput() < ShooterConstants.kSpeakerSpeedRight-0.05) {
            leftMotor.set(ShooterConstants.kSpeakerSpeedLeft);
            rightMotor.set(ShooterConstants.kSpeakerSpeedRight);
            ledIdle = false;
            ledSubsystem.rainbowMode = false;
            ledSubsystem.setColor(0, 0, 255);
        }
        ledSubsystem.setColor(0, 255, 0);
    }

    public void setAmpSpeed() {
        /*while (leftMotor.getAppliedOutput() < ShooterConstants.kShooterMotorLeftSpeed-0.07 && rightMotor.getAppliedOutput() < ShooterConstants.kShooterMotorRightSpeed-0.07) {
            leftPidController.setReference(ShooterConstants.kShooterMotorLeftSpeed, CANSparkMax.ControlType.kVelocity);
            rightPidController.setReference(ShooterConstants.kShooterMotorRightSpeed, CANSparkMax.ControlType.kVelocity);
            ledIdle = false;
            ledSubsystem.rainbowMode = false;
            ledSubsystem.setColor(0, 0, 255);
        }*/
        leftMotor.set(ShooterConstants.kAmpSpeedLeft);
        rightMotor.set(ShooterConstants.kAmpSpeedRight);
        while (leftMotor.getAppliedOutput() < ShooterConstants.kAmpSpeedLeft-0.05 && rightMotor.getAppliedOutput() < ShooterConstants.kAmpSpeedRight-0.05) {
            leftMotor.set(ShooterConstants.kAmpSpeedLeft);
            rightMotor.set(ShooterConstants.kAmpSpeedRight);
            ledIdle = false;
            ledSubsystem.rainbowMode = false;
            ledSubsystem.setColor(0, 0, 255);
        }
        ledSubsystem.setColor(0, 255, 0);
    }

    public void stopShooter() {
        leftMotor.set(0);
        rightMotor.set(0);
        ledIdle = true;
    }

    @Override
    public void periodic() {
        if (ledIdle) {
            if (GlobalVariables.getInstance().extenderFull) {
                ledSubsystem.rainbowMode = false;
                ledSubsystem.setColor(222, 49, 0);
            } else {
                ledSubsystem.rainbowMode = true;
            }
        } else {
            ledSubsystem.rainbowMode = false;
            ledSubsystem.setColor(0, 0, 0);
        }
    }

}
