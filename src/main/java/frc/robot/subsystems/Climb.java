package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
    private final CANSparkMax climbMotor;

    public Climb() {
        climbMotor = new CANSparkMax(ClimbConstants.kClimbMotorId, CANSparkMax.MotorType.kBrushless);
        climbMotor.setInverted(ClimbConstants.kClimbMotorReversed);

        climbMotor.setIdleMode(IdleMode.kBrake);

        climbMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        climbMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        climbMotor.burnFlash();
    }   

    public void setOutputPercentage(double percentage) {
        climbMotor.set(percentage);
    }

    public void setSoftLimitMode(boolean enabled){
        climbMotor.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
    }

    public boolean isAtBottom(){
        return climbMotor.getEncoder().getPosition() <= 0;
    }

    public void stop(){
        climbMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Amperage", climbMotor.getOutputCurrent());
    }
}
