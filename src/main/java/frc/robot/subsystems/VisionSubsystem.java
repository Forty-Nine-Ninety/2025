package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Ports.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RumbleCommandHelper;

public class VisionSubsystem extends SubsystemBase{
    private SwerveSubsystem m_drivebase;
    private PhotonCamera arducamOne;
    
    private PhotonPipelineResult result;
    private PhotonTrackedTarget currentTarget;
    private Transform3d pose;
    int aprilTagiD;
    double nodeLR;

    private RumbleCommandHelper m_rumble;

    public VisionSubsystem(SwerveSubsystem drivebase,PhotonCamera photonCamera,String node,CommandXboxController joystick){
        m_drivebase = drivebase;
        arducamOne = photonCamera;
        nodeLR = (node.equals("left"))?(-VisionConstants.tagToNodeDistance):
                                                VisionConstants.tagToNodeDistance;
        m_rumble = new RumbleCommandHelper(joystick);
    }

    public void update(){
        result = arducamOne.getLatestResult();

        if(result.hasTargets()){
            currentTarget = result.getBestTarget();
            aprilTagiD = currentTarget.getFiducialId(); 
            System.out.printf("Target detected:",aprilTagiD);
            //double distance = sqrt(Math.pow(pose.getX(),2)+Math.pow(pose.getY(),2));
            while(true){
                pose = currentTarget.getBestCameraToTarget();
                double xSpeed = (Math.abs(pose.getX())<nodeLR-VisionConstants.xMarginOfError || pose.getX()>nodeLR+VisionConstants.xMarginOfError)? VisionConstants.translationSpeed:0;
                double ySpeed = (pose.getY()>0.399+VisionConstants.yMarginOfError)? VisionConstants.translationSpeed:0;
                double rotationSpeed = (Math.abs(currentTarget.getYaw())>VisionConstants.rotationMarginOfError)? VisionConstants.rotationSpeed:0;
                if(rotationSpeed!=0 && currentTarget.getYaw()<0){
                    rotationSpeed = -rotationSpeed;
                }
                if(xSpeed==0 && ySpeed==0 && rotationSpeed==0){
                    m_rumble.schedule();
                    break;
                }
                /*NOTE: MIGHT HAVE A PROBLEM WITH DRIVING FOREVER AND NOT UPDATING DATA */
                //pose = currentTarget.getBestCameraToTarget();
                //m_drivebase.drive(new Translation2d(pose.getX(),pose.getY()),currentTarget.getYaw(),true);
            }

            m_drivebase.drive(new Translation2d(pose.getX(),pose.getY()),currentTarget.getYaw(),true);
        }
        else{
            System.out.println("Target not detected");
        }
    }
}

/*NOTE: MEASUREMENTS ARE IN M, M/SEC, RAD/SEC */
/*ADD ABORT COMMAND */