package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakextenderConstants;

public class Extender extends SubsystemBase {
    private TalonSRX extenderMotor;

    public Extender() {
        extenderMotor = new TalonSRX(IntakextenderConstants.kExtenderMotorId);

        extenderMotor.setInverted(IntakextenderConstants.kExtenderMotorReversed);
    }

    public void setOutputPercentage(double percentage) {
        extenderMotor.set(TalonSRXControlMode.PercentOutput, percentage);
    }

    public void stop(){
        extenderMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
