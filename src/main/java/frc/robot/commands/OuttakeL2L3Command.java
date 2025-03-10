package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.L2L3ShooterSubsystem;

public class OuttakeL2L3Command extends Command{
    private final L2L3ShooterSubsystem m_L2L3Shooter;

    public OuttakeL2L3Command (L2L3ShooterSubsystem L2L3shooter){
        m_L2L3Shooter = L2L3shooter;
        addRequirements(L2L3shooter);
    }

    /*@Override
    public void initialize(){
        m_L2L3Shooter.runMotor(0.25);
    }*/

    @Override
    public void execute(){
        m_L2L3Shooter.runMotor(0.4);
    }

    @Override
    public void end(boolean interrupted){
        m_L2L3Shooter.runMotor(0);
    }
}