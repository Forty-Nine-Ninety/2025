package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleCommandHelper extends Command{
    CommandXboxController joystick;

    public RumbleCommandHelper(CommandXboxController joystick){
        this.joystick = joystick;
    }

    @Override
    public void execute(){
        joystick.getHID().setRumble(RumbleType.kBothRumble,1);
    }

    @Override
    public void end(boolean interrupted){
        joystick.getHID().setRumble(RumbleType.kBothRumble,0);
    }
}