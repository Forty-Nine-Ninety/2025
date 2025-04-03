package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.photonvision.PhotonCamera;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class VisionAlignLeftCommand extends Command{
    private PhotonCamera photonCamera;
    private VisionSubsystem m_vision;
    private RobotContainer m_robotContainer;

    public VisionAlignLeftCommand(SwerveSubsystem drivebase,PhotonCamera camera,CommandXboxController joystick,RobotContainer robotContainer){
        photonCamera = camera;
        m_vision = new VisionSubsystem(drivebase,photonCamera,joystick);
        m_vision.setNodeLR("left");
        m_robotContainer = robotContainer;
    }

    @Override
    public void initialize(){
        //m_robotContainer.cancelDriveCommand();
        m_vision.update();
        //m_robotContainer.setTeleopDefaultCommands();
    }

    @Override
    public void end(boolean interrupted){
        //m_robotContainer.cancelDriveCommand();
        //m_robotContainer.setTeleopDefaultCommands();
    }
}