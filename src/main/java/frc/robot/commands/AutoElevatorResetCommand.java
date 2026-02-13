package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoElevatorResetCommand extends Command{
    private final ElevatorSubsystem m_elevator;
    
    public AutoElevatorResetCommand(ElevatorSubsystem elevator){
        m_elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        m_elevator.zeroElevatorPosition();
        System.out.println("Old encoder value: "+m_elevator.getPosition());
    }

    @Override
    public void execute(){
        System.out.println("New encoder value: "+m_elevator.getPosition());
    }

    @Override
    public boolean isFinished(){
        return true;  // Finish after one cycle
    }
}