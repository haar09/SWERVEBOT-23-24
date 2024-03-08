package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class SpeakerShoot extends Command{
    private final Shooter shooter;
    private final Extender extender;
    private final ShooterPivot m_ShooterPivot;
    private boolean ending;
    private double start;

    private final LEDSubsystem ledSubsystem;

    public SpeakerShoot(Shooter shooter, ShooterPivot pivot ,Extender extender, LEDSubsystem ledSubsystem){
        this.shooter = shooter;
        this.m_ShooterPivot = pivot;
        this.extender = extender;
        this.ledSubsystem = ledSubsystem;
        ending = false;
        addRequirements(shooter, extender, ledSubsystem); 
    }

    @Override
    public void initialize(){
        ending = false;
    }

    @Override
    public void execute(){
            m_ShooterPivot.setDesiredAngle(ShooterConstants.kMaxShooterAngleRad);
            shooter.ledIdle = false;
            ledSubsystem.setColor(0, 0, 255);
            shooter.setSpeakerSpeed();
            start = Timer.getFPGATimestamp();
            while (Timer.getFPGATimestamp() - start < 0.3) {
                 extender.setOutputPercentage(-0.5);
            }
            while (Timer.getFPGATimestamp() - start < 1.8) {
                shooter.setSpeakerSpeed();
                extender.setOutputPercentage(1);
            }
            shooter.ledIdle = true;
            ending=true;
    }
    
    @Override
    public void end(boolean interrupted){
        shooter.stopShooter();
        extender.setOutputPercentage(0);
    }

    @Override
    public boolean isFinished(){
        return ending;
    }

}
