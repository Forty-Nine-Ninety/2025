package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.L1ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.OuttakeL1Command;



public class AutoPracticeCommand extends SequentialCommandGroup{
    private final SwerveSubsystem m_drivebase;
    private final L1ShooterSubsystem m_L1shooter;

public AutoPracticeCommand(SwerveSubsystem drivebase, L1ShooterSubsystem shooter, String pathname){

m_drivebase = drivebase;
m_L1shooter = shooter;
addCommands(m_drivebase.getAutonomousCommand(pathname),
    new WaitCommand(0.5),
    new OuttakeL1Command(m_L1shooter).withTimeout(2));

}
}