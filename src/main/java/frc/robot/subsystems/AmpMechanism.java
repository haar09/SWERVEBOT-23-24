package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class AmpMechanism extends SubsystemBase {
    private TalonSRX ampMotor;

    public AmpMechanism() {
        ampMotor = new TalonSRX(ClimbConstants.kAmpMechanismMotorId);

        ampMotor.setInverted(ClimbConstants.kAmpMechanismMotorReversed);

        ampMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setOutputPercentage(double percentage) {
        ampMotor.set(TalonSRXControlMode.PercentOutput, percentage * ClimbConstants.kAmpMechanismMotorMultiplier);
    }

    public void stop(){
        ampMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
