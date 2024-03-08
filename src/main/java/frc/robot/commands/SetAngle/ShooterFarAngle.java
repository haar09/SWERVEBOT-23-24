package frc.robot.commands.SetAngle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterPivot;

public class ShooterFarAngle extends Command{
    private final ShooterPivot m_ShooterPivot;

    public ShooterFarAngle(ShooterPivot shooter) {
        this.m_ShooterPivot = shooter;
        addRequirements(m_ShooterPivot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_ShooterPivot.setDesiredAngle(Math.toRadians(15));
    }

    public void end(boolean interrupted) {
        RobotContainer.rumble(0, 0, 0, 0);
        m_ShooterPivot.setDesiredAngle(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}