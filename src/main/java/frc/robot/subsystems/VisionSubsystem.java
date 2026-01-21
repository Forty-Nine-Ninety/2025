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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.RumbleCommandHelper;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.VisionDriveCommand;
import frc.robot.commands.auto.AutoVisionCommand;
import java.util.List;

public class VisionSubsystem extends SubsystemBase{
    private SwerveSubsystem m_drivebase;
    private VisionDriveCommand m_visionDrive;
    private DriveCommand m_driveCommand;
    private PhotonCamera arducamOne;
    
    private List<PhotonPipelineResult> results;
    private PhotonTrackedTarget currentTarget;
    private RumbleCommand m_rumble;
    private CommandXboxController m_joystick;
    private Transform3d m_pose;
    private double m_lastTargetTime;
    private double[] m_speeds;
    private Transform3d m_prevPose;
    private boolean driving = false;

    public VisionSubsystem(SwerveSubsystem drivebase,PhotonCamera photonCamera,CommandXboxController joystick){
        m_drivebase = drivebase;
        arducamOne = photonCamera;
        m_rumble = new RumbleCommand(joystick);
        m_joystick = joystick;
        m_pose = new Transform3d();
        m_lastTargetTime = 0;
        m_driveCommand = new DriveCommand(m_drivebase);
        m_speeds = new double[3]; // [xSpeed,ySpeed,rotationSpeed]
        
    } 

    //This method finds the distance of the April Tag being scanned
    public void scanForApriltag(){
        results = arducamOne.getAllUnreadResults();
        
        if (results.size()!=0 && results!= null){
            PhotonPipelineResult result = results.get(results.size()-1);
            if(result.hasTargets()&&!driving){
                m_pose = result.getBestTarget().getBestCameraToTarget();
            } else {
                m_pose = new Transform3d();
            }
        } else {
            m_pose = new Transform3d();
        }
        results.clear();
    }
    //Updates the variables in Vision Subsystem. Run this method often.
    public void update(){
        this.scanForApriltag();
        double poseX = m_pose.getX();
        double poseY = m_pose.getY();
        double poseYaw = m_pose.getRotation().getZ()*180/Math.PI;

        //GET SPEEDS
        //System.out.println("pose: "+poseX+" "+poseY+" "+poseYaw);
        if (poseX>1){
            m_speeds[0] = VisionConstants.maxTranslationSpeed;
        } else if (poseX>0.3){
            m_speeds[0] = 0.5*poseX;
        } else if(Timer.getFPGATimestamp()-m_lastTargetTime>0.0005){
            m_speeds[0] = 0;
        }

        if (poseY>0.5){
            m_speeds[1] = VisionConstants.maxTranslationSpeed;
        } else if (poseY>0.15){
            m_speeds[1] = 0.5*poseY;
        } else if (Timer.getFPGATimestamp()-m_lastTargetTime>0.0005){
            m_speeds[1] = 0;
        }

        if(poseYaw!=0){
            if (Math.abs(180-Math.abs(poseYaw))>45){
                m_speeds[2] = VisionConstants.maxRotationSpeed;
            } else if (Math.abs(180-Math.abs(poseYaw))>1){
                m_speeds[2] = 0.5*Math.abs(180-Math.abs(poseYaw))/90*VisionConstants.maxRotationSpeed;
            }
        } else if(Timer.getFPGATimestamp()-m_lastTargetTime>0.0005){
            m_speeds[2] = 0;
        }
        //System.out.println("time: "+m_lastTargetTime+" "+Timer.getFPGATimestamp());
        System.out.println("speeds: "+m_speeds[0]+" "+m_speeds[1]+" "+m_speeds[2]);

        m_drivebase.drive(new ChassisSpeeds(m_speeds[0],m_speeds[1],m_speeds[2]));
        if(poseX>0.3 || poseY>0.15 || poseYaw>1){
            m_lastTargetTime = Timer.getFPGATimestamp();
        }
    }
}