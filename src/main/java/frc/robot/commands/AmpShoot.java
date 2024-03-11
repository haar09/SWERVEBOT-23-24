package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.GlobalVariables;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;

public class AmpShoot extends Command{
    private final Supplier<Double> rightTrigger;
    private final Shooter shooter;
    private final Extender extender;
    private final ColorSensor colorSensor;
    private boolean ending;
    private double start;

    private final LEDSubsystem ledSubsystem;

    public AmpShoot(Supplier<Double> rightTrigger ,Shooter shooter, Extender extender, LEDSubsystem ledSubsystem, ColorSensor colorSensor){
        this.shooter = shooter;
        this.extender = extender;
        this.ledSubsystem = ledSubsystem;
        this.rightTrigger = rightTrigger;
        this.colorSensor = colorSensor;
        ending = false;
        addRequirements(shooter, extender, ledSubsystem); 
    }

    @Override
    public void initialize(){
        ending = false;
    }

    @Override
    public void execute(){
            shooter.ledIdle = false;
            ledSubsystem.setColor(0, 0, 255);
            shooter.setAmpSpeed();
            /*while (rightTrigger.get() < 0.3) {
                RobotContainer.rumble(0, 0, 0, 0);
                //wait
            }*/
            if (rightTrigger.get() > 0.3) {
                if (colorSensor.getSensor()) {
                    RobotContainer.rumble(0, 0, 0, 0);
                }  else {
                    RobotContainer.rumble(1,1, 1, 1);
                }
                    start = Timer.getFPGATimestamp();
                    while (Timer.getFPGATimestamp() - start < 0.3) {
                        extender.setOutputPercentage(-0.5);
                    }
                    while (Timer.getFPGATimestamp() - start < 2.5) {
                        extender.setOutputPercentage(1);
                    }
                    ending=true;
                    end(false);
                    extender.setOutputPercentage(0);
                    shooter.stopShooter();
            } else {
                RobotContainer.rumble(0, 0, 0, 0);
            }
    }
    
    @Override
    public void end(boolean interrupted){
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished(){
        return ending;
    }

}
