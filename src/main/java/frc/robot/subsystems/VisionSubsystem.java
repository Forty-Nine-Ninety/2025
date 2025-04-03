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
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.RumbleCommandHelper;
import frc.robot.commands.VisionDriveCommand;
import frc.robot.commands.auto.AutoVisionCommand;

public class VisionSubsystem extends SubsystemBase{
    private SwerveSubsystem m_drivebase;
    private DriveCommand m_driveCommand;
    private PhotonCamera arducamOne;
    
    private PhotonPipelineResult result;
    private PhotonTrackedTarget currentTarget;
    private Transform3d pose;
    int aprilTagiD;
    double nodeLR;

    private RumbleCommand m_rumble;
    private CommandXboxController m_joystick;

    public VisionSubsystem(SwerveSubsystem drivebase,PhotonCamera photonCamera,CommandXboxController joystick){
        m_drivebase = drivebase;
        arducamOne = photonCamera;
        nodeLR = -VisionConstants.tagToNode;
        m_rumble = new RumbleCommand(joystick);
        m_joystick = joystick;
        m_driveCommand = new DriveCommand(m_drivebase);
    }

    public void setNodeLR(String node){
        nodeLR = (node.equals("left"))?(-VisionConstants.tagToNode):
                                                 VisionConstants.tagToNode;
    }

    public void scanForApriltag(){
        PhotonPipelineResult result = arducamOne.getLatestResult();
        if(result.hasTargets()){
          Transform3d pose = result.getBestTarget().getBestCameraToTarget();
          double distance = Math.sqrt(Math.pow(pose.getX(),2)+Math.pow(pose.getY(),2));
          if(distance<=3){
            //new RumbleCommand(m_joystick);
            //m_rumble.schedule();
          }
        }
    }

    public void update(){
        result = arducamOne.getLatestResult();
        boolean done = false;

        if(result.hasTargets()){
            currentTarget = result.getBestTarget();
            //double distance = sqrt(Math.pow(pose.getX(),2)+Math.pow(pose.getY(),2));
            //while(!done&&result.hasTargets()){
                drive();
                System.out.println("driving");
                /*pose = currentTarget.getBestCameraToTarget();
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
                    //m_rumble.schedule();
                    //new RumbleCommand(m_joystick);
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
                m_driveCommand.setSuppliers(pose.getX(),pose.getY(),currentTarget.getYaw());
                System.out.println("SET SUPPLIERS");
                new VisionDriveCommand(m_driveCommand);
                System.out.println("SCHEDULED");
                
                //new AutoVisionCommand(m_drivebase,pose,currentTarget);
                //break;
                */
            //}
        }
        else{
            System.out.println("4. Target not detected");
        }
    }

    private boolean drive(){
        if(!result.hasTargets()){
            return false;
        }
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
                    System.out.println("SPEEDS ZERO");
                    return true;
                }
                System.out.println(pose);
                System.out.println(xSpeed);
                //else{
                    //m_driveCommand.cancel();
                    m_driveCommand.setSuppliers(pose.getX(),pose.getY(),currentTarget.getYaw());
                    new VisionDriveCommand(m_driveCommand);
                    return false;
                //}
    }

    /*public void visionDrive(){
        while(!drive()){
            System.out.println("driving");
          }
    }*/

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