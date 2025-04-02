package frc.robot.commands.auto;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.DriveUtil;


public class AutoDriveCommand extends Command{
    private final SwerveSubsystem m_drivebase;
    private final Transform3d pose;
    private final PhotonTrackedTarget currentTarget;

    public AutoDriveCommand(SwerveSubsystem drivebase,Transform3d pose,PhotonTrackedTarget currentTarget){
        m_drivebase = drivebase;
        this.pose = pose;
        this.currentTarget = currentTarget;
    }

    @Override
    public void initialize(){
         m_drivebase.drive(new Translation2d(DriveUtil.powCopySign(pose.getY(),3),
                                    DriveUtil.powCopySign(pose.getX(),3)),
                                    DriveUtil.powCopySign(currentTarget.getYaw(),3),false);
    }
}
