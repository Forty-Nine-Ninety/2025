package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class VisionAlignCommand {
    private final SwerveSubsystem m_drivebase;
    public VisionAlignCommand(SwerveSubsystem drivebase){
        m_drivebase = drivebase;
        addCommands(new )
    }

}




public class Auto21NBlueCommand extends SequentialCommandGroup{
    private final SwerveSubsystem m_drivebase;
    private final ArmSubsystem m_arm;
    private final ShooterSubsystem m_shooter;

    public Auto21NBlueCommand(SwerveSubsystem drivebase, ArmSubsystem arm, ShooterSubsystem shooter) {
        m_drivebase = drivebase;
        m_arm = arm;
        m_shooter = shooter;
        addCommands(new AutoSpeakerCommand(m_shooter, m_arm).withTimeout(4.05),
                    new WaitCommand(0),
                    m_drivebase.getAutonomousCommand("21NBlue")
        );
    }
}
