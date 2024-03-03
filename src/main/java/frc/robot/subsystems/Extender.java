package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakextenderConstants;

public class Extender extends SubsystemBase {
    private CANSparkMax extenderMotor;

    public Extender() {
        extenderMotor = new CANSparkMax(IntakextenderConstants.kExtenderMotorId, CANSparkMax.MotorType.kBrushless);

        extenderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        extenderMotor.setInverted(IntakextenderConstants.kExtenderMotorReversed);
    }

    public void setOutputPercentage(double percentage) {
        extenderMotor.set(percentage);
    }

    public void stop(){
        extenderMotor.set(0);
    }
}
