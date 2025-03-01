/*COMMENT package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.vision.pipeline.result.CVPipelineResult;
import org.photonvision.PhotonPoseEstimator;
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
        nodeLR = (node.equals("left"))?-Constants.VisionConstants.tagToNodeDistance:
                                        Constants.VisionConstants.tagToNodeDistance;
    }

    public void update(){
        result = camera.getLatestResult();

        if(result.hasTargets()){
            currentTarget = result.getBestTarget();
            aprilTagiD = currentTarget.getFiducialId(); 
            //double distance = sqrt(Math.pow(pose.getX(),2)+Math.pow(pose.getY(),2));
            while(true){
                pose = currentTarget.bestCameratoTarget();
                double xSpeed = (Math.abs(pose.getX)<nodeLR-Constants.VisionConstants.xMarginOfError || pose.getX>nodeLR+Constants.VisionConstants.xMarginOfError)? 1:0;
                double ySpeed = (pose.getY>0.399+Constants.VisionConstants.yMarginOfError)? Constants.VisionConstants.ySpeed:0;
                double rotationSpeed = (Math.abs(currentTarget.getYaw)>Constants.VisionConstants.rotationMarginOfError)? Constants.VisionConstnats.rotationSpeed:0;
                if(rotationSpeed!=0 && currentTarget.getYaw<0){
                    rotationSpeed = -rotationSpeed;
                }
                if(xSpeed==0 && ySpeed==0 && rotationSpeed==0){
                    break;
                    /*Rumble here */
//COMMENT                }
                /*NOTE: MIGHT HAVE A PROBLEM WITH DRIVING FOREVER AND NOT UPDATING DATA */
/*COMMENT                m_drivebase.drive(new ChassisSpeeds(xSpeed,ySpeed,rotationSpeed),
                                  false,
                                  new Translation2d(0,0));
            }
        }
    }
}

COMMENT*/

/*NOTE: MEASUREMENTS ARE IN M, M/SEC, RAD/SEC */
/*ADD ABORT COMMAND */