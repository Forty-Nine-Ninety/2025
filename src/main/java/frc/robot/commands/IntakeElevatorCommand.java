import frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subssytems.L2L3ShooterSubsystem;

public class IntakeElevatorCommand extends Command{
    private final ElevatorSubsystem m_elevator;
    private final L2L3ShooterSubsystem m_shooter;

    public IntakeElevatorCommand(ElevatorSubsystem elevator, L2L3ShooterSubsystem shooter){
        m_elevator = elevator;
        m_shooter = shooter;
        addRequirements(elevator);
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        m_elevator.moveToPosition(/*tbd */);
    }

    @Override
    public void execute(){
        if(m_shooter.breakBeam()){
            cancel();
        }
        else{
            m_shooter.runMotor(1);
        }
    }
    @Override
    public void end(boolean interrupted){
        m_shooter.runMotor(0);
    }
}

 