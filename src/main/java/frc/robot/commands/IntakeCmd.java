package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.IntakextenderConstants;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends Command{
    private final Intake intake;
    private final Extender extender;
    private final Supplier<Boolean> take, out, hard;
    private final XboxController driverController;

    public IntakeCmd(Supplier<Boolean> take, Supplier<Boolean> out, Supplier<Boolean> hard,
    Intake intake, Extender extender, XboxController driverController){
        this.take = take;
        this.out = out;
        this.hard = hard;
        this.intake = intake;
        this.extender = extender;
        this.driverController = driverController;
        addRequirements(intake);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        // get joystick
        boolean takeSpeed = take.get();
        boolean outSpeed = out.get();

        if (takeSpeed) {
            if (hard.get()) {
                intake.setOutputPercentage(0.8);
                extender.setOutputPercentage(0.8);
            } else {
                if (!GlobalVariables.getInstance().extenderFull) {
                    intake.setOutputPercentage(IntakextenderConstants.kIntakeMotorSpeed);
                    extender.setOutputPercentage(IntakextenderConstants.kExtenderSpeed);
                } else {
                    driverController.setRumble(RumbleType.kBothRumble, 0.6);
                }
            }
        } else if (outSpeed) {
            if (hard.get()) {
                intake.setOutputPercentage(-0.8);
                extender.setOutputPercentage(-0.8);
            } else {
                intake.setOutputPercentage(-IntakextenderConstants.kIntakeMotorSpeed);
                extender.setOutputPercentage(-IntakextenderConstants.kExtenderSpeed);
            }

        }
        else {
            driverController.setRumble(RumbleType.kBothRumble, 0);
            intake.setOutputPercentage(0);
            extender.setOutputPercentage(0);
        }
        if (GlobalVariables.getInstance().extenderFull && !outSpeed && !hard.get()) {
            intake.setOutputPercentage(0);
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