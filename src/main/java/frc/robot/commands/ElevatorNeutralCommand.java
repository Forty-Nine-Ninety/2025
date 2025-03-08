package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorNeutralCommand extends Command{
    private final ElevatorSubsystem m_elevator;

    public ElevatorNeutralCommand(ElevatorSubsystem elevator){
        m_elevator = elevator;
        addRequirements(elevator);
    }
    
    @Override
    public void initialize(){
        m_elevator.moveToPosition(0);
    }
}