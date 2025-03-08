package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class IntakeElevatorCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final L2L3ShooterSubsystem m_shooter;

    public IntakeElevatorCommand(ElevatorSubsystem elevator, L2L3ShooterSubsystem L2L3Shooter){
        m_elevator = elevator;
        m_shooter = L2L3Shooter;
        addRequirements(elevator);
        addRequirements(L2L3Shooter);
    }

    @Override
    public void initialize(){
        m_elevator.moveToPosition(3.3);
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

 