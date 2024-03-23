package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;

public class IntakeIn extends Command{
    private final Intake intake;
    private final Extender extender;
    private boolean ending;
    private final ColorSensor colorSensor;

    public IntakeIn(Intake intake, Extender extender, ColorSensor colorSensor){
        this.intake = intake;
        this.extender = extender;
        this.colorSensor = colorSensor;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        ending = false;
    }

    @Override
    public void execute(){
        if (!colorSensor.getSensor()) {
        intake.setOutputPercentage(0.3);
        extender.setOutputPercentage(0.4);
        } else {
            System.out.println("help");
            intake.setOutputPercentage(0);
            extender.setOutputPercentage(0);
            ending = true;
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return ending;
    }
}