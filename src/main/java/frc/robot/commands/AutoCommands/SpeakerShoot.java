package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.ShooterPivot;

public class SpeakerShoot extends Command{
    private final Shooter shooter;
    private final Extender extender;
    private final ShooterPivot m_ShooterPivot;
    private boolean ending;
    private double start;
    private double desiredAngle;

    private final LEDSubsystem ledSubsystem;

    public SpeakerShoot(Shooter shooter, ShooterPivot pivot ,Extender extender, LEDSubsystem ledSubsystem, double desiredAngle){
        this.shooter = shooter;
        this.m_ShooterPivot = pivot;
        this.extender = extender;
        this.ledSubsystem = ledSubsystem;
        this.desiredAngle = desiredAngle;
        ending = false;
        addRequirements(shooter, m_ShooterPivot, extender, ledSubsystem); 
    }

    @Override
    public void initialize(){
        ending = false;
        shooter.state = ShooterState.IDLE;
    }

    @Override
    public void execute(){
            shooter.setSpeakerSpeed();
            if (desiredAngle == 0) {
                desiredAngle = GlobalVariables.getInstance().speakerToAngle();
            }
            m_ShooterPivot.setDesiredAngle(desiredAngle);
            ledSubsystem.setColor(0, 0, 255);
            if (shooter.state == ShooterState.READY) {
                start = Timer.getFPGATimestamp();
                while (Timer.getFPGATimestamp() - start < 0.05) {
                    extender.setOutputPercentage(-IntakextenderConstants.kExtenderBackSpeed);
                    m_ShooterPivot.setDesiredAngle(desiredAngle);
                }
                while (Timer.getFPGATimestamp() - start < 1.1) {
                    shooter.setSpeakerSpeed();
                    extender.setOutputPercentage(1);
                    m_ShooterPivot.setDesiredAngle(desiredAngle);
                }
                ending=true;
            }
    }
    
    @Override
    public void end(boolean interrupted){
        shooter.state = ShooterState.IDLE;
        shooter.stopShooter();
        extender.setOutputPercentage(0);
    }

    @Override
    public boolean isFinished(){
        return ending;
    }

}
