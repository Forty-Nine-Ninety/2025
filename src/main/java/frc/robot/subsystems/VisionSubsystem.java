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
    private VisionDriveCommand m_visionDrive;
    private DriveCommand m_driveCommand;
    private PhotonCamera arducamOne;
    
    private PhotonPipelineResult result;
    private PhotonTrackedTarget currentTarget;
    private Transform3d pose;
    int aprilTagiD;
    double nodeLR;

    private RumbleCommand m_rumble;
    private CommandXboxController m_joystick;
    private boolean driving = false;

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
        if(result.hasTargets()&&!driving){
          Transform3d pose = result.getBestTarget().getBestCameraToTarget();
          double distance = Math.sqrt(Math.pow(pose.getX(),2)+Math.pow(pose.getY(),2));
          if(distance<=3){
            new RumbleCommand(m_joystick);
            m_rumble.schedule();
          }
        }
    }

    public void update(){
        result = arducamOne.getLatestResult();
        driving = true;
        if(result.hasTargets()){currentTarget = result.getBestTarget();}
        else{
            System.out.println("Target not detected");
            return;
        }
        m_visionDrive = new VisionDriveCommand(this,m_drivebase,result,currentTarget,nodeLR);
        //m_visionDrive.wait(200);
        System.out.println("driving");
    }

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