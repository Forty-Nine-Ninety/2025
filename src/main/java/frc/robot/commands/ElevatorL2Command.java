package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorL2Command extends Command{
// instantiate the class so it transfers from elevator subsystem
    private final ElevatorSubsystem m_elevator;
    public ElevatorL2Command (ElevatorSubsystem m_elevator){
        m_elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void initialize(){
        elevator.moveToPosition(/*tbd*/);

    }
}