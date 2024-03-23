package frc.robot.commands.SetAngle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class ShooterAngle extends Command{
    private final ShooterPivot m_ShooterPivot;

    public ShooterAngle(ShooterPivot shooter) {
        this.m_ShooterPivot = shooter;
        addRequirements(m_ShooterPivot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_ShooterPivot.setDesiredAngle(29); //subwoofer dibi
    }

    public void end(boolean interrupted) {
        m_ShooterPivot.setDesiredAngle(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}