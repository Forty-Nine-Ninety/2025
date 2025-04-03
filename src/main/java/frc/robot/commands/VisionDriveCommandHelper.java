package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class VisionDriveCommandHelper extends Command{
    private DriveCommand m_driveCommand;

    public VisionDriveCommandHelper(DriveCommand driveCommand){
        m_driveCommand = driveCommand;
    }

    @Override
    public void initialize(){
        System.out.println("VISION HELPER");
        m_driveCommand.schedule();
    }

    @Override
    public void end(boolean interrupted){
        m_driveCommand.cancel();
    }
}