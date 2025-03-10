package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * Climber steps 1.1 - 1.3 should autosequence (TODO):
 * 1.1 Lift trapdoor
 * 1.2 Lift elevator to SPECIAL_CLIMB_POSITION
 * 1.3 Deploy stinger (rotation)
 * (driver will drive robot into position to engage CAGE)
 * 2. Rotate stinger to CAGE_POSITION
 * 3. Bring elevator to zero
 * 4. Engage clips
 */
public class ElevatorUnclimbCommand extends Command{
    private final ClimberSubsystem m_climber;

    
    public ElevatorUnclimbCommand(ClimberSubsystem climber){
        m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize(){
        m_climber.reverseClimbState();

    }
}