package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class IntakeElevatorCommand extends Command {
    private final L2L3ShooterSubsystem m_shooter;

    public IntakeElevatorCommand(L2L3ShooterSubsystem L2L3Shooter){
        m_shooter = L2L3Shooter;
        addRequirements(L2L3Shooter);
    }

    @Override
    public void execute(){
        if(m_shooter.breakBeam()){
            cancel();
        }
        else{
            m_shooter.runMotor(0.6);
        }
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.runMotor(0);
    }
}

 