package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorNeutralCommand extends SequentialCommandGroup{
    private final ElevatorSubsystem m_elevator;

    public ElevatorNeutralCommand(ElevatorSubsystem elevator){
        m_elevator = elevator;
        addCommands(new AutoElevatorZeroCommand(m_elevator),
                    new WaitCommand(0),
                    new AutoElevatorResetCommand(m_elevator));
    }
}