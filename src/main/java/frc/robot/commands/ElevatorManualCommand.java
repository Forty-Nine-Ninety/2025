package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
public class ElevatorManualCommand extends Command{
    private final ElevatorSubsystem m_elevator;
    public ElevatorNeutralCommand(ElevatorSubsystem elevator){
        m_elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void moveToPosition(double setPoint){
        m_elevator.moveToPosition(setPoint);
    }
}