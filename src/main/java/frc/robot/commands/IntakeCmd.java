package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends Command{
    private final Intake intake;
    private final Extender extender;
    private final Supplier<Double> take, out;
    private final Supplier<Boolean> hard;

    public IntakeCmd(Supplier<Double> take, Supplier<Double> out, Supplier<Boolean> hard,Intake intake, Extender extender){
        this.take = take;
        this.out = out;
        this.hard = hard;
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

        if (takeSpeed > IntakextenderConstants.kIntakeDeadband) {
            if (hard.get()) {
                intake.setOutputPercentage(0.8);
            } else {
                intake.setOutputPercentage(IntakextenderConstants.kIntakeMotorSpeed);
            }
            if (!GlobalVariables.getInstance().extenderFull) {
                if (hard.get()) {
                    extender.setOutputPercentage(0.8);
                } else {
                    extender.setOutputPercentage(IntakextenderConstants.kIntakeMotorSpeed);
                }
            }

        } else if (outSpeed > IntakextenderConstants.kIntakeDeadband) {
            if (hard.get()) {
                intake.setOutputPercentage(-0.8);
                extender.setOutputPercentage(-0.8);
            } else {
                intake.setOutputPercentage(-IntakextenderConstants.kIntakeMotorSpeed);
                extender.setOutputPercentage(-IntakextenderConstants.kIntakeMotorSpeed);
            }

        }
         else {
            intake.setOutputPercentage(0);
            extender.setOutputPercentage(0);
        }
        if (GlobalVariables.getInstance().extenderFull && outSpeed < IntakextenderConstants.kIntakeDeadband) {
            extender.setOutputPercentage(0);
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