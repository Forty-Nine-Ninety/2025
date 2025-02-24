
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.L1ShooterSubsystem;
import frc.robot.subsystems.L2L3ShooterSubsystem;
public class OuttakeL2L3Command {
    private final L2L3ShooterSubsystem m_L2L3Shooter;
    public OuttakeL2L3Command (L2L3ShooterSubsystem L2L3Shooter){
        m_L2L3Shooter = L2L3Shooter;
        addRequirements(L2L3Shooter);
    }
    @Override
    public void initialize(){
        m_L2L3Shooter.runMotor(1);
    }

}
