package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class Degree180Turn extends Command{    
    private final SwerveSubsystem swerveSubsystem;
    private double currentAngle;
    private double targetAngle;

    public Degree180Turn(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
    }

    PIDController thetaController = new PIDController(PIDConstants.kP180Rotate, 0, PIDConstants.kD180Rotate);
    @Override
    public void initialize(){
        currentAngle = swerveSubsystem.getHeading();
        targetAngle = currentAngle + 360;
        targetAngle %= 360;
        targetAngle -= 180;
        System.out.println(targetAngle);
    }

    @Override
    public void execute(){
        currentAngle = swerveSubsystem.getHeading();
        GlobalVariables.getInstance().customRotateSpeed = thetaController.calculate(currentAngle, targetAngle);
    }

    public void end(boolean interrupted){
        GlobalVariables.getInstance().customRotateSpeed = 0;
        thetaController.close();
    }

    @Override
    public boolean isFinished(){
        if (currentAngle >= (targetAngle-10+180)%360-180 && currentAngle <= (targetAngle+10+180)%360-180){
            return true;
        }
        return false;
    }
}
