package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.L1ShooterSubsystem;
public class L1ShooterSubsystem extends Command{
// instantiate the class so it transfers from elevator subsystem
    private final L1ShooterSubsystem m_L1Shooter;
    public ElevatorL1Command (L1ShooterSubsystem m_L1Shooter){
        m_L1Shooter = L1Shooter;
        addRequirements(L1Shooter);
    }
    @Override
        public void initialize(){
            L1Shooter.runMotor(1);
    }
}