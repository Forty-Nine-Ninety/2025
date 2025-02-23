package frc.robot.subsystems;

import edu.wpi.first.photonvision.PhotonCamera;
import edu.wpi.first.photonvision.PhotonPipelineResult;
import edu.wpi.first.photonvision.PhotonPoseEstimator;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class VisionSubsystem extends SubsystemBase{
    private SwerveSubsystem m_drivebase;
    private PhotonCamera arducamOne;
    
    private PhotonPipelineResult result;
    private PhotonTrackedTarget currentTarget;
    private Transform3D pose;
    int aprilTagiD;
    int nodeLR;

    public VisionSubsystem(SwerveSubsystem drivebase,PhotonCamera photonCamera,String node){
        m_drivebase = drivebase;
        arducamOne = photonCamera;
        nodeLR = (node.equals("left"))?-Constants.VisionConstants.tagToNodeDistance,
                                        Constants.VisionConstants.tagToNodeDistance;
    }

    public void update(){
        result = camera.getLatestResult();

        if(result.hasTargets()){
            currentTarget = result.getBestTarget();
            aprilTagiD = currentTarget.getFiducialId();
            pose = currentTarget.bestCameratoTarget();
            double distance = sqrt(Math.pow(pose.getX(),2)+Math.pow(pose.getY(),2));
            while(pose.getY>0.399){
                double xSpeed = (Math.abs(pose.getX)<nodeLR-Constants.AutonConstants.x || pose.getX>nodeLR+0.05)? 1:0;
                double ySpeed = (pose.getY>0.399+margin)? 1:0;
                double rotationSpeed = (Math.abs(currentTarget.getYaw)>0.1)? 3:0;
                if( rotationSpeed!=0 && currentTarget.getYaw<0){
                    rotationSpeed = -rotationSpeed;
                }
                m_drivebase.drive(new ChassisSpeeds(xSpeed,ySpeed,rotationSpeed),
                                  false,
                                  new Translation2d(0,0));
            }
        }
    }
}

/*NOTE: MEASUREMENTS ARE IN M, M/SEC, RAD/SEC */