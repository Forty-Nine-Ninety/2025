package frc.robot.commands;

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.photonvision.PhotonCamera;
import frc.robot.subsystems.SwerveSubsystem;

public class VisionAlignLeftCommand extends Command{
    private PhotonCamera photonCamera;
    private VisionSubsystem m_vision;

    public VisionAlignLeftCommand(SwerveSubsystem drivebase,String camera,CommandXboxController joystick){
        photonCamera = new PhotonCamera(camera);
        m_vision = new VisionSubsystem(drivebase,photonCamera,"left",joystick);
    }

    @Override
    public void initialize(){
        m_vision.update();
    }
}