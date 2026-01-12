package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Ports.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.List;

//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class VisionDriveCommand extends Command{

    private VisionSubsystem m_vision;

    private DriveCommand m_driveCommand;
    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget currentTarget;
    private Transform3d pose;
    private double nodeLR;

    public VisionDriveCommand(VisionSubsystem vision, SwerveSubsystem drivebase,
                              PhotonPipelineResult result,PhotonTrackedTarget currentTarget,
                              double nodeLR){
        m_vision = vision;
        this.result = result;
        this.currentTarget = currentTarget;
        this.nodeLR = nodeLR;
        m_driveCommand = new DriveCommand(drivebase);
        addRequirements(vision);
    }

    @Override
    public void initialize(){

        if(!result.hasTargets()){
            System.out.println("NO TARGETS");
        }
        pose = currentTarget.getBestCameraToTarget();
        System.out.println(pose.getX() + "" + pose.getY());
        double xSpeed = (pose.getX()<nodeLR-VisionConstants.xMarginOfError || 
                         pose.getX()>nodeLR+VisionConstants.xMarginOfError)?
                         m_vision.getTranslationSpeed(pose.getX()):0; //real life y axis
        double ySpeed = (pose.getY()>0.399+VisionConstants.yMarginOfError)?
                         m_vision.getTranslationSpeed(pose.getY()):0; //real life x axis
        double rotationSpeed = (Math.abs(currentTarget.getYaw())>VisionConstants.rotationMarginOfError)? VisionConstants.rotationSpeed:0;
        if(rotationSpeed!=0 && currentTarget.getYaw()<0){
            rotationSpeed = -rotationSpeed;
        }
        if(xSpeed==0 && ySpeed==0 && rotationSpeed==0){
            System.out.println("SPEEDS ZERO");
            return;
        }
        m_driveCommand.setSuppliers(pose.getX(),pose.getY(),currentTarget.getYaw());
        m_driveCommand.schedule();
    }
}