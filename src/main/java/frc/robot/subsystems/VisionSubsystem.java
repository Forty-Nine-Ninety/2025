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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Ports.DriveSettings;
import frc.robot.Constants.Ports.VisionConstants;
import frc.robot.DriveUtil;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import java.util.List;

public class VisionSubsystem extends SubsystemBase{
    private SwerveSubsystem m_drivebase;
    private VisionDriveCommand m_visionDrive;
    private DriveCommand m_driveCommand;
    private PhotonCamera arducamOne;
    
    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget currentTarget;
    int aprilTagiD;
    double nodeLR;

    private RumbleCommand m_rumble;
    private CommandXboxController m_joystick;
    private Transform3d m_pose;
    private Transform3d m_prevPose;
    private boolean driving = false;

    public VisionSubsystem(SwerveSubsystem drivebase,PhotonCamera photonCamera,CommandXboxController joystick){
        m_drivebase = drivebase;
        arducamOne = photonCamera;
        nodeLR = -VisionConstants.tagToNode;
        m_rumble = new RumbleCommand(joystick);
        m_joystick = joystick;
        m_pose = new Transform3d(0,0,0,new Rotation3d());
        m_driveCommand = new DriveCommand(m_drivebase);
        
    } 

    public void setNodeLR(String node){
        nodeLR = (node.equals("left"))?(-VisionConstants.tagToNode):
                                                 VisionConstants.tagToNode;
    }
    //This method finds the distance of the April Tag being scanned
    public void scanForApriltag(){
        List<PhotonPipelineResult> results = arducamOne.getAllUnreadResults();
        
        if (results.size()!=0 && results != null){
            PhotonPipelineResult result = results.get(results.size()-1);
            if(result.hasTargets()&&!driving){
                m_pose = result.getBestTarget().getBestCameraToTarget();
                System.out.println(m_pose);
            }
        }
            //List<PhotonTrackedTarget> targetPositions = result.getTargets();
    }
    //Updates the variables in Vision Subsystem. Run this method often.
    public void update(){
        this.scanForApriltag();
        double xSpeed,ySpeed,rotationSpeed = 0;
        double x = m_pose.getX();
        double y = m_pose.getY();
        double z = m_pose.getZ();
        double yaw = m_pose.getRotation().getZ()*180/Math.PI;
        double distance = Math.sqrt(Math.pow(m_pose.getX(),2)+Math.pow(m_pose.getY(),2));
        //System.out.println("x:"+pose.getX()+" ; y:"+pose.getY()+" ; z:"+pose.getZ()+" ; yaw:"+pose.getRotation().getZ()*180/Math.PI);
        if(distance<=3){
            new RumbleCommand(m_joystick);
            m_rumble.schedule();
        }
        //GET SPEEDS
        //get x speed
        if (m_pose.getX()>1){xSpeed = 4.5;}
        else if (m_pose.getX()>0.1524){xSpeed = 0.5*pose.getX();} // FIGURE OUT SPEEDS
        else{xSpeed = 0;}
        //get y speed
        if (m_pose.getY()>1){ySpeed = 4.5;}
        else if (m_pose.getY()>0.1524){ySpeed = 0.5*pose.getY();} // FIGURE OUT SPEEDS
        else{ySpeed = 0;}
        rotationSpeed = 0;
        m_drivebase.drive(new ChassisSpeeds(xSpeed,ySpeed,rotationSpeed));
        //m_visionDrive = new VisionDriveCommand(this,m_drivebase,result,currentTarget,nodeLR);
        //m_visionDrive.wait(200);
        System.out.println("x: "+xSpeed+" y: "+ySpeed+" z: "+rotationSpeed);
    }

    // Getter methods. They return data. Only run after using update()
    public Transform3d getPose(){
        return pose;
    }
    // public (Unknown Data Type) getAngle(){
    //     return angle;
    // }


    // The ultimate goal, driving the robot to the vision tag once alligned. Before we work on this we need to potentially rework vision but at least fix it.
    private void drive(){
        if(!result.hasTargets()){
            return;
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
            return;
           }
        //m_driveCommand.setSuppliers(pose.getX(),pose.getY(),currentTarget.getYaw());
        //new VisionDriveCommand(m_driveCommand);
    }

    public double getTranslationSpeed(double distance){
        double speed = 2.90/(1+4.75*Math.pow(Math.E,41)*Math.pow(Math.E,(-248*distance)));
        return speed;
    }
}