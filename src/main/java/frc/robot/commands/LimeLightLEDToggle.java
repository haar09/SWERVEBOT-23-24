package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LimeLight;

public class LimeLightLEDToggle extends InstantCommand{
    private final LimeLight m_LimeLight;
    public LimeLightLEDToggle(LimeLight subsystem){
        m_LimeLight = subsystem;
        addRequirements(m_LimeLight);
    }

    @Override
    public void initialize(){
        if(m_LimeLight.getLedMode() == false){
            m_LimeLight.setLedMode(true);
        }else{
            m_LimeLight.setLedMode(false);
        }
    }
}
