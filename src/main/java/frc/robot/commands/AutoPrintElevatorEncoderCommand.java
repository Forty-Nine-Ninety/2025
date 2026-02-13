package frc.robot.commands;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
public class AutoPrintElevatorEncoderCommand extends Command{
    private final ElevatorSubsystem m_elevator;

    public AutoPrintElevatorEncoderCommand(ElevatorSubsystem elevator){
        m_elevator = elevator;
    }
    

    @Override
    public void execute(){
        System.out.println(m_elevator.getPosition());
    }

}
