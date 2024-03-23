package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.GlobalVariables;

public class Shooter extends SubsystemBase{

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    /*private final SparkPIDController leftPidController;
    private final SparkPIDController rightPidController;*/

    private final LEDSubsystem ledSubsystem;

    public boolean ledIdle;
    public double time;

    public Shooter(LEDSubsystem m_LedSubsystem) {
        ledSubsystem = m_LedSubsystem;

        leftMotor = new CANSparkMax(ShooterConstants.kShooterMotorLeftId, CANSparkMax.MotorType.kBrushless);
        rightMotor = new CANSparkMax(ShooterConstants.kShooterMotorRightId, CANSparkMax.MotorType.kBrushless);

        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftMotor.setInverted(ShooterConstants.kShooterMotorLeftReversed);
        rightMotor.setInverted(ShooterConstants.kShooterMotorRightReversed);

        leftMotor.enableVoltageCompensation(10.9);
        rightMotor.enableVoltageCompensation(10.9);

        leftMotor.burnFlash();
        rightMotor.burnFlash();

        ledIdle = true;
    }

    public void setSpeakerSpeed() {
        SmartDashboard.putBoolean("shooterReady", false);
        ledIdle = false;
        leftMotor.set(ShooterConstants.kSpeakerSpeedLeft);
        rightMotor.set(ShooterConstants.kSpeakerSpeedRight);
        time = Timer.getFPGATimestamp();
        while (leftEncoder.getVelocity() < 3400 && rightEncoder.getVelocity() < 2900) {
            if (Timer.getFPGATimestamp() - time > 2) {
                stopShooter();
                return;
            }
            leftMotor.set(ShooterConstants.kSpeakerSpeedLeft);
            rightMotor.set(ShooterConstants.kSpeakerSpeedRight);
        }
        ledSubsystem.setColor(0, 255, 0);
        SmartDashboard.putBoolean("shooterReady", true);
    }

    public void setAmpSpeed() {
        SmartDashboard.putBoolean("shooterReady", false);
        ledIdle = false;
        leftMotor.set(ShooterConstants.kAmpSpeedLeft);
        rightMotor.set(ShooterConstants.kAmpSpeedRight);
        while (leftEncoder.getVelocity() < 600 && rightEncoder.getVelocity() < 600) {
            leftMotor.set(ShooterConstants.kAmpSpeedLeft);
            rightMotor.set(ShooterConstants.kAmpSpeedRight);
        }
        ledSubsystem.setColor(0, 255, 0);
        SmartDashboard.putBoolean("shooterReady", true);
    }

    public void stopShooter() {
        leftMotor.set(0);
        rightMotor.set(0);
        ledIdle = true;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("leftSpeed", leftEncoder.getVelocity());
        SmartDashboard.putNumber("rightSpeed", rightEncoder.getVelocity());
        if (ledIdle) {
            if (GlobalVariables.getInstance().extenderFull) {
                ledSubsystem.rainbowMode = false;
                if (GlobalVariables.getInstance().speakerToAngle() > 0) {
                    ledSubsystem.setColor(0, 0, 255);
                } else {
                    ledSubsystem.setColor(222, 49, 0);
                }
            } else {
                ledSubsystem.rainbowMode = true;
            }
        } else {
            ledSubsystem.rainbowMode = false;
        }
    }

}
