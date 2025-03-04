package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorClimbCommand extends Command{
    private final ElevatorSubsystem m_elevator;

    public ElevatorClimbCommand(ElevatorSubsystem elevator){
        m_elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        m_elevator.moveToPosition(30);

    }
}