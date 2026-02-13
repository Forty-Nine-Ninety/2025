package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.util.Color;

public class VisionDetectionCommand extends Command{
    private final BlinkinSubsystem m_blinkin;
    private final VisionSubsystem m_vision;
    

    public VisionDetectionCommand(BlinkinSubsystem blinkin, VisionSubsystem vision){
        m_blinkin = blinkin;
        m_vision = vision;
        addRequirements(m_blinkin, m_vision);
        
    }
    @Override
    public void initialize(){
        if(m_vision.isInIdealRange()){
            m_blinkin.setColor(m_blinkin.green);
        }
        else{
            m_blinkin.setColor(m_blinkin.red);
        }
        
    }
}
