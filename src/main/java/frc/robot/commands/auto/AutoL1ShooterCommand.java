package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.L1ShooterSubsystem;
import frc.robot.commands.OuttakeL1Command;


public class AutoL1ShooterCommand extends SequentialCommandGroup{
    private final L1ShooterSubsystem m_shooter;

    public AutoL1ShooterCommand(L1ShooterSubsystem shooter){
        m_shooter = shooter;
        addCommands(new OuttakeL1Command(m_shooter).withTimeout(2.5));
    }
}
