package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.L1ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.OuttakeL1Command;

public class Auto1NBlueCommand extends SequentialCommandGroup{
    private final SwerveSubsystem m_drivebase;
    private final L1ShooterSubsystem m_L1shooter;

    //exit
    public Auto1NBlueCommand(SwerveSubsystem drivebase, L1ShooterSubsystem shooter,String pathname,String exitPath) {
        m_drivebase = drivebase;
        m_L1shooter = shooter;
        addCommands(m_drivebase.getAutonomousCommand(pathname),
                    //new WaitCommand(0),
                    new OuttakeL1Command(m_L1shooter).withTimeout(2),
                    //new WaitCommand(0)
                    m_drivebase.getAutonomousCommand(exitPath)
        );
    }

    //No exit
    public Auto1NBlueCommand(SwerveSubsystem drivebase, L1ShooterSubsystem shooter,String pathname) {
        m_drivebase = drivebase;
        m_L1shooter = shooter;
        addCommands(m_drivebase.getAutonomousCommand(pathname),
                    //new WaitCommand(0),
                    new OuttakeL1Command(m_L1shooter).withTimeout(2)
        );
    }
}
