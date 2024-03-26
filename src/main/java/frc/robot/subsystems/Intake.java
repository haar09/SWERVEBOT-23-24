package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakextenderConstants;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(IntakextenderConstants.kIntakeMotorId, CANSparkMax.MotorType.kBrushless);
        intakeMotor.setInverted(IntakextenderConstants.kIntakeMotorReversed);

        intakeMotor.setIdleMode(IdleMode.kCoast);

        intakeMotor.burnFlash();
    }   

    public void setOutputPercentage(double percentage) {
        intakeMotor.set(percentage);
    }

    public void stop(){
        intakeMotor.set(0);
    }
}
