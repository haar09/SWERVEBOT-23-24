package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LimeLight;

public class RotateToTargetWhileDrive extends Command{
    private final LimeLight m_LimeLight;
    private double tx;
    
    public RotateToTargetWhileDrive(LimeLight subsystem){
        this.m_LimeLight = subsystem;
        addRequirements(m_LimeLight);
    }

    PIDController thetaController = new PIDController(PIDConstants.kPLimeLightRotate, 0, 0.000018);
    @Override
    public void initialize(){
        System.out.println("START");
    }

    @Override
    public void execute(){
        if (m_LimeLight.getTargetID() != 7 && m_LimeLight.getTargetID()!= 3){
        tx = -m_LimeLight.getTX();
        GlobalVariables.getInstance().rotateToTargetSpeed = thetaController.calculate(0, tx);
        }
    }

    public void end(boolean interrupted){
        GlobalVariables.getInstance().rotateToTargetSpeed = 0;
        thetaController.close();
    }

    @Override
    public boolean isFinished(){
        return Math.abs(tx) < VisionConstants.kTXTolerance;
    }
}
