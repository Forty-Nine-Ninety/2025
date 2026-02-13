package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoElevatorZeroCommand extends Command{
    private final ElevatorSubsystem m_elevator;
    
    public AutoElevatorZeroCommand(ElevatorSubsystem elevator){
        m_elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        System.out.println("Old encoder value: "+m_elevator.getPosition());
        m_elevator.moveToPosition(0);
        new WaitCommand(5);
        System.out.println("New encoder value: "+m_elevator.getPosition());
        
    }
}