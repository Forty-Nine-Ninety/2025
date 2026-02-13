package frc.robot.commands.auto;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;


public class AutoVisionCommand extends SequentialCommandGroup{
    private SwerveSubsystem m_drivebase;
    private Transform3d m_pose;
    private PhotonTrackedTarget m_currentTarget;

    public AutoVisionCommand(SwerveSubsystem drivebase,Transform3d pose,PhotonTrackedTarget currentTarget){
        m_drivebase = drivebase;
        m_pose = pose;
        m_currentTarget = currentTarget;
        System.out.println("RUNNING");
        System.out.println(m_pose);
        addCommands(new AutoDriveCommand(m_drivebase,m_pose,m_currentTarget));
    }
}
