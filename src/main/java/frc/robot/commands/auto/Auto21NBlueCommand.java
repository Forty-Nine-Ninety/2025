package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto21NBlueCommand extends SequentialCommandGroup{
    private final SwerveSubsystem m_drivebase;
    public Auto21NBlueCommand(SwerveSubsystem drivebase){
        m_drivebase = drivebase;
        addCommands(m_drivebase.drive(new ChassisSpeeds(x,y,theta), rotation, true));
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