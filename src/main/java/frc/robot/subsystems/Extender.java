package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakextenderConstants;

public class Extender extends SubsystemBase {
    private final CANSparkMax extenderMotor;

    public Extender() {
        extenderMotor = new CANSparkMax(IntakextenderConstants.kExtenderMotorId, CANSparkMax.MotorType.kBrushless);
        extenderMotor.setInverted(IntakextenderConstants.kExtenderMotorReversed);

        extenderMotor.setIdleMode(IdleMode.kCoast);

        extenderMotor.burnFlash();
    }

    public void setOutputPercentage(double percentage) {
        extenderMotor.set(percentage);
    }

    public void stop(){
        extenderMotor.set(0);
    }
}
