package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.L1ShooterSubsystem;
public class OuttakeL1Command extends Command{
// instantiate the class so it transfers from elevator subsystem
    private final L1ShooterSubsystem m_L1Shooter;
    public OutakeL1Command (L1ShooterSubsystem L1ShooterSubsystem){
        m_L1Shooter = L1ShooterSubsytem;
        addRequirements(L1ShooterSubsytem);
    }
    @Override
        public void initialize(){
            m_L1Shooter.runMotor(1);
    }
}