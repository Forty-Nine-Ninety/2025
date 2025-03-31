package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.photonvision.PhotonCamera;
import frc.robot.subsystems.SwerveSubsystem;

public class VisionAlignRightCommand extends Command{
    private PhotonCamera photonCamera;
    private VisionSubsystem m_vision;

    public VisionAlignRightCommand(SwerveSubsystem drivebase,PhotonCamera camera,CommandXboxController joystick){
        photonCamera = camera;
        m_vision = new VisionSubsystem(drivebase,photonCamera,joystick);
        m_vision.setNodeLR("right");
    }

    @Override
    public void initialize(){
        m_vision.update();
    }
}