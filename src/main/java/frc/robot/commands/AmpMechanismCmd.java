package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AmpMechanism;

public class AmpMechanismCmd extends Command{

    private final AmpMechanism ampMechanism;
    private final Supplier<Double> xSpdFunc;
    
    public AmpMechanismCmd(AmpMechanism ampMechanism, Supplier<Double> xSpdFunc){
        this.ampMechanism = ampMechanism;
        this.xSpdFunc = xSpdFunc;
        addRequirements(ampMechanism);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        // get joystick
        double xSpeed = xSpdFunc.get();
        
        // deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0;
        
        ampMechanism.setOutputPercentage(xSpeed);
    }

    @Override
    
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
