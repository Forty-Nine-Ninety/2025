
//IMPORT STATEMENTS
import frc.robot.commands;

public class IntakeElevatorCommand extends Command{
    //INSTANCE VARIABLES
    private final ElevatorSubsystem m_elevator;
    private final L2L3ShooterSubsystem m_shooter;

    //CONSTRUCTOR
    public IntakeElevatorCommand(ElevatorSubsystem elevator, L2L3ShooterSubsystem shooter){
        m_elevator = elevator;
        m_shooter = shooter;
        addRequirements(elevator);
        addRequirements(shooter);
    }
    @Override
    public void initialize(){
        m_elevator.moveToPosition(0);
    }

    @Override
    public void execute(){
        if(m_shooter.breakBeam()){
            cancel();
        }
        else{
            m_shooter.runMotor(50);
        }
    }
    @Override
    public void end(boolean interrupted){
        m_shooter.runMotor(0);
    }
}

 