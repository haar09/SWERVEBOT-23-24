package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.LimeLight;

public class LimeLightRotateToTarget extends CommandBase{
    private final LimeLight m_LimeLight;
    public LimeLightRotateToTarget(LimeLight subsystem){
        this.m_LimeLight = subsystem;
        addRequirements(m_LimeLight);
    }

    PIDController thetaController = new PIDController(PIDConstants.kPLimeLightRotate, 0, 0);

    @Override
    public void initialize(){
        m_LimeLight.setLedMode(true);
    }

    @Override
    public void execute(){
        double tx = -m_LimeLight.getTX();

        GlobalVariables.getInstance().rotateToTargetSpeed = thetaController.calculate(0, tx);
    }

    public void end(boolean interrupted){
        GlobalVariables.getInstance().rotateToTargetSpeed = 0;
        thetaController.close();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
