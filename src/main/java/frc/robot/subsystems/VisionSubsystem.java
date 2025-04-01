package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Ports.DriveSettings;
import frc.robot.Constants.Ports.VisionConstants;
import frc.robot.DriveUtil;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RumbleCommandHelper;
import frc.robot.commands.auto.AutoVisionCommand;

public class VisionSubsystem extends SubsystemBase{
    private SwerveSubsystem m_drivebase;
    private PhotonCamera arducamOne;
    
    private PhotonPipelineResult result;
    private PhotonTrackedTarget currentTarget;
    private Transform3d pose;
    int aprilTagiD;
    double nodeLR;

    private RumbleCommandHelper m_rumble;

    public VisionSubsystem(SwerveSubsystem drivebase,PhotonCamera photonCamera,CommandXboxController joystick){
        m_drivebase = drivebase;
        arducamOne = photonCamera;
        nodeLR = -VisionConstants.tagToNode;
        m_rumble = new RumbleCommandHelper(joystick);
    }

    public void scanForApriltag(){
        PhotonPipelineResult result = arducamOne.getLatestResult();
        if(result.hasTargets()){
          Transform3d pose = result.getBestTarget().getBestCameraToTarget();
          double distance = Math.sqrt(Math.pow(pose.getX(),2)+Math.pow(pose.getY(),2));
          System.out.println(pose);
          System.out.println(distance);
          if(distance<=3){
            update();
          }
        }
      }

    public void setNodeLR(String node){
        nodeLR = (node.equals("left"))?(-VisionConstants.tagToNode):
                                                 VisionConstants.tagToNode;
    }

    public void update(){
        System.out.println("3. Update");
        result = arducamOne.getLatestResult();

        if(result.hasTargets()){
            currentTarget = result.getBestTarget();
            aprilTagiD = currentTarget.getFiducialId(); 
            System.out.printf("4. Target detected:",aprilTagiD);
            //double distance = sqrt(Math.pow(pose.getX(),2)+Math.pow(pose.getY(),2));
            //while(true){
                pose = currentTarget.getBestCameraToTarget();
                double xSpeed = (pose.getX()<nodeLR-VisionConstants.xMarginOfError || 
                                 pose.getX()>nodeLR+VisionConstants.xMarginOfError)?
                                 getTranslationSpeed(pose.getX()):0; //real life y axis
                double ySpeed = (pose.getY()>0.399+VisionConstants.yMarginOfError)?
                                 getTranslationSpeed(pose.getY()):0; //real life x axis
                double rotationSpeed = (Math.abs(currentTarget.getYaw())>VisionConstants.rotationMarginOfError)? VisionConstants.rotationSpeed:0;
                if(rotationSpeed!=0 && currentTarget.getYaw()<0){
                    rotationSpeed = -rotationSpeed;
                }
                if(xSpeed==0 && ySpeed==0 && rotationSpeed==0){
                    m_rumble.schedule();
                    //break;
                }
                //System.out.println("5. Speeds:");
                //System.out.println(xSpeed);
                //System.out.println(ySpeed);
                //System.out.println(rotationSpeed);
                //System.out.println(pose.getX());
                //System.out.println(pose.getY());
                //System.out.println(currentTarget.getYaw());
                //Pose2d targetPose = new Pose2d(pose.getX(),pose.getY(),new Rotation2d(currentTarget.getYaw()*2*Math.PI/180));
                //m_drivebase.driveToPose(new Pose2d(pose.getX(),pose.getY(),new Rotation2d(currentTarget.getYaw()*2*Math.PI/180)));
                //System.out.println(targetPose);
                //m_drivebase.drive(new Translation2d(DriveUtil.powCopySign(pose.getY(),3),
                //                    DriveUtil.powCopySign(pose.getX(),3)),
                //                    DriveUtil.powCopySign(currentTarget.getYaw(),3),false);
                //m_drivebase.drive(new ChassisSpeeds(ySpeed,xSpeed,rotationSpeed));
                new AutoVisionCommand(m_drivebase,pose,currentTarget);
                System.out.println("6. REPEAT");
                //break;
            //}
        }
        else{
            System.out.println("4. Target not detected");
        }
    }

    private double getTranslationSpeed(double distance){
        double speed = 2.90/(1+4.75*Math.pow(Math.E,41)*Math.pow(Math.E,(-248*distance)));
        return speed;
    }
}

/*NOTE: MEASUREMENTS ARE IN M, M/SEC, RAD/SEC */
/*ADD ABORT COMMAND */
//Get camera
//Drive to position
//Shoot coral