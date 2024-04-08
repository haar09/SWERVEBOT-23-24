package frc.robot.commands.SetAngle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ShooterPivot;

public class ShooterSetAngle extends Command{
    private final ShooterPivot m_ShooterPivot;

    public ShooterSetAngle(ShooterPivot shooter) {
        this.m_ShooterPivot = shooter;
        addRequirements(m_ShooterPivot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_ShooterPivot.setDesiredAngle(SmartDashboard.getNumber("customAngle", VisionConstants.y_ArmAngle[0]));
    }

    public void end(boolean interrupted) {
        m_ShooterPivot.setDesiredAngle(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}