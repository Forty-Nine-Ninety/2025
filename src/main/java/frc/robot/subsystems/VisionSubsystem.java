package frc.robot.subsystems;

import edu.wpi.first.photonvision.PhotonCamera;
import edu.wpi.first.photonvision.PhotonPipelineResult;
import edu.wpi.first.photonvision.PhotonPoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class VisionSubsystem {
    private SwerveSubsystem m_drivebase;
    private PhotonCamera arducamOne;
    //private PhotonPoseEstimator poseEstimator;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget currentTarget;
    private Transform2D pose;
    int aprilTagiD;

    public VisionSubsystem(SwerveSubsystem drivebase){
        m_drivebase = new SwerveSubsystem(drivebase);
        arducamOne = new PhotonCamera("Arducam 1");
    }

    public void update(){
        result = camera.getLatestResult();

        if(result.hasTargets()){
            currentTarget = result.getBestTarget();
            //poseEstimator.update();
            aprilTagiD = currentTarget.getFiducialId();
            pose = currentTarget.getCameraToTarget();
            //USE DATA TO DRIVE ROBOT TO CORRECT POSITION, AUTO COMMAND
            m_drivebase.drive();
            pose.getX();
            pose.getY();
            
        }
    }
}