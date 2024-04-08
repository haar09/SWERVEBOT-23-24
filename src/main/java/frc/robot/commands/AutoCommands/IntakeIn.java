package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;

public class IntakeIn extends Command{
    private final Intake intake;
    private final Extender extender;
    private boolean ending;

    public IntakeIn(Intake intake, Extender extender){
        this.intake = intake;
        this.extender = extender;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        ending = false;
    }

    @Override
    public void execute(){
        if (!GlobalVariables.getInstance().extenderFull) {
        intake.setOutputPercentage(IntakextenderConstants.kIntakeMotorSpeed);
        extender.setOutputPercentage(IntakextenderConstants.kExtenderSpeed);
        } else {
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