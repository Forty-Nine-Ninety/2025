package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RumbleCommand extends SequentialCommandGroup{

    CommandXboxController m_joystick;

    public RumbleCommand(CommandXboxController joystick){
        m_joystick = joystick;
        addCommands(new ParallelRaceGroup(new RumbleCommandHelper(m_joystick),
                                          new WaitCommand(1)));
    }
}
