package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends Command{
    private final Intake intake;
    private final Extender extender;
    private final Supplier<Double> take, out;

    public IntakeCmd(Supplier<Double> take, Supplier<Double> out, Intake intake, Extender extender){
        this.take = take;
        this.out = out;
        this.intake = intake;
        this.extender = extender;
        addRequirements(intake);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        // get joystick
        double takeSpeed = take.get();
        double outSpeed = out.get();

        if (takeSpeed > 0 || outSpeed > 0) {
            intake.setOutputPercentage(takeSpeed - outSpeed);
            if (!GlobalVariables.getInstance().extenderFull || takeSpeed < outSpeed) {
                extender.setOutputPercentage(takeSpeed - outSpeed);
            }
        } else {
            intake.setOutputPercentage(0);
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