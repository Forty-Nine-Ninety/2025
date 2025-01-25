package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorL1Command extends Command{
    private final ElevatorSubsystem m_elevator;
    public ArmAmpCommand(ElevatorSubsystem m_elevator){
        m_elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void initialize(){
        elevator.moveToPosition(tbd);

    }
}