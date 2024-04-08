package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.GlobalVariables;

public class Shooter extends SubsystemBase{

    private final TalonFX leftMotor;
    private final VoltageOut leftMotorVoltageRequest;

    private final TalonFX rightMotor;
    private final VoltageOut rightMotorVoltageRequest;

    private final LEDSubsystem ledSubsystem;

    public boolean ledIdle;
    public double time;

    public Shooter(LEDSubsystem m_LedSubsystem) {
        ledSubsystem = m_LedSubsystem;

        leftMotor = new TalonFX(ShooterConstants.kShooterMotorLeftId);
        rightMotor = new TalonFX(ShooterConstants.kShooterMotorRightId);

        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);

        leftMotor.setInverted(ShooterConstants.kShooterMotorLeftReversed);
        rightMotor.setInverted(ShooterConstants.kShooterMotorRightReversed);

        leftMotorVoltageRequest = new VoltageOut(0);
        rightMotorVoltageRequest = new VoltageOut(0);

        ledIdle = true;
    }

    public enum ShooterState {
        IDLE,
        ACCELERATING,
        READY
    }

    public ShooterState state = ShooterState.IDLE;
    private double startTime;

    public void setSpeakerSpeed() {
        switch (state) {
            case IDLE:
                SmartDashboard.putBoolean("shooterReady", false);
                ledIdle = false;
                leftMotor.setControl(leftMotorVoltageRequest.withOutput((ShooterConstants.kSpeakerSpeedLeft * ShooterConstants.kVoltageCompensation)));
                rightMotor.setControl(rightMotorVoltageRequest.withOutput((ShooterConstants.kSpeakerSpeedRight * ShooterConstants.kVoltageCompensation)));
                startTime = Timer.getFPGATimestamp();
                state = ShooterState.ACCELERATING; 
                ledSubsystem.setColor(0, 0, 0);
                break;
            case ACCELERATING:
                if (leftMotor.getVelocity().getValueAsDouble() >= 72 && rightMotor.getVelocity().getValueAsDouble() >= 60) {
                    ledSubsystem.setColor(0, 255, 0);
                    SmartDashboard.putBoolean("shooterReady", true);
                    state = ShooterState.READY;
                } else if (Timer.getFPGATimestamp() - startTime > 2) {
                    stopShooter();
                    state = ShooterState.IDLE;
                }
            break;
            case READY:
                leftMotor.setControl(leftMotorVoltageRequest.withOutput(ShooterConstants.kSpeakerSpeedLeft * ShooterConstants.kVoltageCompensation));
                rightMotor.setControl(rightMotorVoltageRequest.withOutput(ShooterConstants.kSpeakerSpeedRight * ShooterConstants.kVoltageCompensation));
                break;
        }
    }

    public void setAmpSpeed() {
        switch (state) {
            case IDLE:
                SmartDashboard.putBoolean("shooterReady", false);
                ledIdle = false;    
                leftMotor.setControl(leftMotorVoltageRequest.withOutput((ShooterConstants.kAmpSpeedLeft * ShooterConstants.kVoltageCompensation)));
                rightMotor.setControl(rightMotorVoltageRequest.withOutput((ShooterConstants.kAmpSpeedRight * ShooterConstants.kVoltageCompensation)));
                startTime = Timer.getFPGATimestamp();
                state = ShooterState.ACCELERATING; 
                break;
            case ACCELERATING:
                if (leftMotor.getVelocity().getValueAsDouble() >= 27 && rightMotor.getVelocity().getValueAsDouble() >= 26) {
                    ledSubsystem.setColor(0, 255, 0);
                    SmartDashboard.putBoolean("shooterReady", true);
                    state = ShooterState.READY;
                } else if (Timer.getFPGATimestamp() - startTime > 2) {
                    stopShooter();
                    state = ShooterState.IDLE;
                }
            break;
            case READY:
                leftMotor.setControl(leftMotorVoltageRequest.withOutput(ShooterConstants.kAmpSpeedLeft * ShooterConstants.kVoltageCompensation));
                rightMotor.setControl(rightMotorVoltageRequest.withOutput((ShooterConstants.kAmpSpeedRight * ShooterConstants.kVoltageCompensation)));
                break;
        }
    }

    public void stopShooter() {
        leftMotor.setControl(leftMotorVoltageRequest.withOutput(0));
        rightMotor.setControl(leftMotorVoltageRequest.withOutput(0));
        ledIdle = true;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("leftSpeed", leftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("rightSpeed", rightMotor.getVelocity().getValueAsDouble());
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
