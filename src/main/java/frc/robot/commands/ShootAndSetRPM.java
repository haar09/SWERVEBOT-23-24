package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;

public class ShootAndSetRPM extends Command{
    private final Supplier<Double> rightTrigger;
    private final Supplier<Boolean> triangleButton;
    private final Shooter shooter;
    private final Extender extender;

    private final LEDSubsystem ledSubsystem;

    public ShootAndSetRPM(Supplier<Double> rightTrigger, Supplier<Boolean> triangleButton ,Shooter shooter, Extender extender, LEDSubsystem ledSubsystem){
        this.shooter = shooter;
        this.extender = extender;
        this.ledSubsystem = ledSubsystem;
        this.rightTrigger = rightTrigger;
        this.triangleButton = triangleButton;
        addRequirements(shooter, extender, ledSubsystem); 
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        if (triangleButton.get()) {
            shooter.ledIdle = false;
            ledSubsystem.setColor(0, 0, 255);
            shooter.setShooterSpeed();
            while (rightTrigger.get() < 0.3) {
                RobotContainer.rumble(0, 0, 0, 0);
                //wait
            }
            if (rightTrigger.get() > 0.3) {
                if (GlobalVariables.getInstance().extenderFull) {
                    RobotContainer.rumble(0, 0, 0, 0);
                    while (GlobalVariables.getInstance().extenderFull) {
                        extender.setOutputPercentage(1);
                    }
                    extender.setOutputPercentage(0);
                    shooter.stopShooter();
                    end(false);
                } else {
                    RobotContainer.rumble(1,1, 1, 1);
                }
            } else {
                RobotContainer.rumble(0, 0, 0, 0);
            }
        } else {
            shooter.stopShooter();
        }
    }
    
    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
