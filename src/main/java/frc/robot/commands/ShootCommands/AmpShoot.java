package frc.robot.commands.ShootCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AmpShoot extends Command{
    private final Supplier<Double> rightTrigger;
    private final Shooter shooter;
    private final Extender extender;
    private final Intake intake;
    private boolean ending;
    private double start;

    public AmpShoot(Supplier<Double> rightTrigger ,Shooter shooter, Intake intake,Extender extender){
        this.shooter = shooter;
        this.extender = extender;
        this.intake = intake;
        this.rightTrigger = rightTrigger;
        ending = false;
        addRequirements(shooter, extender); 
    }

    @Override
    public void initialize(){
        ending = false;
    }

    @Override
    public void execute(){
            shooter.ledIdle = false;
            shooter.setAmpSpeed();
            if (rightTrigger.get() > 0.3) {
                    start = Timer.getFPGATimestamp();
                    while (Timer.getFPGATimestamp() - start < 0.15) {
                        extender.setOutputPercentage(-0.5);
                    }
                    while (Timer.getFPGATimestamp() - start < 1.3) {
                        shooter.setAmpSpeed();
                        extender.setOutputPercentage(1);
                        intake.setOutputPercentage(0.6);
                    }
                    ending=true;
                    end(false);
                    extender.setOutputPercentage(0);
                    shooter.stopShooter();
            }
    }
    
    @Override
    public void end(boolean interrupted){
        intake.setOutputPercentage(0);
        extender.setOutputPercentage(0);
        shooter.stopShooter();    
        SmartDashboard.putBoolean("shooterReady", false);
    }

    @Override
    public boolean isFinished(){
        return ending;
    }

}
