package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class VisionDriveCommand extends SequentialCommandGroup{

    public VisionDriveCommand(DriveCommand m_driveCommand){
        System.out.println("VISION DRIVE COMMAND");
        addCommands(new VisionDriveCommandHelper(m_driveCommand).withTimeout(1));
    }
}