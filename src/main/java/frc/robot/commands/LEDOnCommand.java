package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
public class LEDOnCommand extends Command{
    private final BlinkinSubsystem m_blinkin;
    public LEDOnCommand(){
        m_blinkin = new BlinkinSubsystem();
    }

    public void initalize(){
        m_blinkin.turnOnLED();
    }

    
}
