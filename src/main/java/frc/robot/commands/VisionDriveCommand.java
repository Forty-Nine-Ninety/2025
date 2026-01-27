package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class VisionDriveCommand extends Command{
    private final VisionSubsystem m_visionSubsystem;
    private final CommandXboxController m_joystick;

    public VisionDriveCommand(VisionSubsystem visionSubsystem,CommandXboxController joystick) {
        m_visionSubsystem = visionSubsystem;
        m_joystick = joystick;
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize(){
        m_visionSubsystem.update();
    }

    /*
    @Override
    public void execute(){
        m_visionSubsystem.update();
    }
    */

    @Override
    public void end(boolean interrupted){
        new RumbleCommand(m_joystick);
    }
}
