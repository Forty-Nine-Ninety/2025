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
    
    private PhotonPipelineResult result;
    private PhotonTrackedTarget currentTarget;
    private Transform2D pose;
    int aprilTagiD;
    int nodeLR;

    public VisionSubsystem(SwerveSubsystem drivebase,String node){
        m_drivebase = new SwerveSubsystem(drivebase);
        arducamOne = new PhotonCamera("Arducam 1");
        nodeLR = (node.equals("left"))?-position,position;
    }

    public void update(){
        result = camera.getLatestResult();

        if(result.hasTargets()){
            currentTarget = result.getBestTarget();
            aprilTagiD = currentTarget.getFiducialId();
            pose = currentTarget.getCameraToTarget();
            double distance = sqrt(Math.pow(pose.getX(),2)+Math.pow(pose.getY(),2));
            while(distance>/*DISTANCE FROM APRILTAG */){
                double xSpeed = (Math.abs(pose.getX)<nodeLR-/*MARGIN OF ERROR*/ || pose.getX>nodeLR+/*MARGIN OF ERROR */)? /*CHASSIS SPEED */:0;
                double ySpeed = (pose.getY>/*DISTANCE FROM CORAL */)? /*CHASSIS SPEED */:0;
                double rotationSpeed = (Math.abs(currentTarget.getYaw)>/*MARGIN OF ERROR */)? /*ROTATION SPEED */:0;
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