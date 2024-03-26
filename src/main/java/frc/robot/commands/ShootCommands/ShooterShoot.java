package frc.robot.commands.ShootCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class ShooterShoot extends Command{
    private final Supplier<Double> rightTrigger;
    private final Shooter shooter;
    private final Extender extender;
    private final Intake intake;
    private boolean ending;

    public ShooterShoot(Supplier<Double> rightTrigger ,Shooter shooter, Intake intake,Extender extender){
        this.shooter = shooter;
        this.extender = extender;
        this.intake = intake;
        this.rightTrigger = rightTrigger;
        ending = false;
        addRequirements(shooter, extender); 
    }

    private enum State {
        START,
        EXTEND,
        SHOOT,
        END
    }

    private State state = State.START;
    private double startTime;

    @Override
    public void initialize(){
        ending = false;
        state = State.START;
        shooter.state = ShooterState.IDLE;
    }

    @Override
    public void execute() {
        double timeElapsed = Timer.getFPGATimestamp() - startTime;
        shooter.setSpeakerSpeed();

        switch (state) {
            case START:
                if (rightTrigger.get() > 0.3 && shooter.state == ShooterState.READY) {
                    startTime = Timer.getFPGATimestamp();
                    state = State.EXTEND;
                }
                break;
            case EXTEND:
                if (timeElapsed < 0.15) {
                    extender.setOutputPercentage(-0.5);
                } else {
                    state = State.SHOOT;
                }
                break;
            case SHOOT:
                if (timeElapsed < 1.3) {
                    shooter.setSpeakerSpeed();
                    extender.setOutputPercentage(1);
                    intake.setOutputPercentage(0.6);
                } else {
                    state = State.END;
                }
                break;
            case END:
                ending = true;
                end(false);
                extender.setOutputPercentage(0);
                shooter.stopShooter();
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted){
        state = State.START;
        shooter.state = ShooterState.IDLE;
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
