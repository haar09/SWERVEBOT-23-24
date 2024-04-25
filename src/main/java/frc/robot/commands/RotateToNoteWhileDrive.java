package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.ObjectDetection;

public class RotateToNoteWhileDrive extends Command{
        
    private double angle;
    private ObjectDetection detector;
    private XboxController driverController;
    
     PIDController thetaController = new PIDController(PIDConstants.kPObjectRotate, 0, PIDConstants.kDObjectRotate);

    public RotateToNoteWhileDrive(ObjectDetection objectDetector, XboxController driverController){
        this.detector = objectDetector;
        this.driverController = driverController;
        thetaController.enableContinuousInput(0, 360);
    }
    
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        angle = detector.getTX();

        GlobalVariables.getInstance().customRotateSpeed = thetaController.calculate(angle);

        if (Math.abs(thetaController.getPositionError()) < 6){
            driverController.setRumble(RumbleType.kBothRumble, 0.15);
        }         
    }

    public void end(boolean interrupted){
        driverController.setRumble(RumbleType.kBothRumble, 0);
        GlobalVariables.getInstance().customRotateSpeed = 0;
        thetaController.close();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}