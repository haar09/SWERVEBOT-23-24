package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbCmd extends Command{
    private final Climb climb;
    private final Supplier<Boolean> take, out, hard;
    private final XboxController operatorController;

    public ClimbCmd(Supplier<Boolean> take, Supplier<Boolean> out, Supplier<Boolean> hard,
    Climb climb, XboxController operatorController){
        this.take = take;
        this.out = out;
        this.hard = hard;
        this.climb = climb;
        this.operatorController = operatorController;
        addRequirements(climb);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        // get joystick
        boolean takeSpeed = take.get();
        boolean outSpeed = out.get();
        boolean hardSpeed = hard.get();

        if (takeSpeed) {
            climb.setOutputPercentage(0.73);
        } else if (outSpeed) {
            if (climb.isAtBottom()) {
                operatorController.setRumble(RumbleType.kBothRumble ,0.4);
            } else {
                operatorController.setRumble(RumbleType.kBothRumble, 0);
                climb.setOutputPercentage(-0.73);
            }
        } else if (hardSpeed) {
                climb.setSoftLimitMode(false);
                climb.setOutputPercentage(-0.73);
        } else {
            climb.setSoftLimitMode(true);
            climb.setOutputPercentage(0);
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