package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorNeutralCommand extends ParallelRaceGroup{
    private final ElevatorSubsystem m_elevator;

    public ElevatorNeutralCommand(ElevatorSubsystem elevator){
        m_elevator = elevator;
        addCommands(new ParallelRaceGroup(new AutoElevatorZeroCommand(m_elevator), new AutoPrintElevatorEncoderCommand(m_elevator)));
                    //new WaitCommand(0.5),
                    //new AutoElevatorResetCommand(m_elevator)
                    //);
                    System.out.println(m_elevator.getPosition());        
        new WaitCommand(0.5) ;   
        m_elevator.zeroElevatorPosition();
        System.out.println(m_elevator.getPosition());
        
    }
}