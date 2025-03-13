package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
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
public class ElevatorClimbCommand extends Command{
    private ClimberSubsystem m_climber;
    // private Debouncer m_debouncer;
    
    public ElevatorClimbCommand(ClimberSubsystem climber){
        m_climber = climber;

        // Creates a Debouncer in "both" mode.
        //m_debouncer = new Debouncer(0.2, Debouncer.DebounceType.kBoth);

        addRequirements(climber);
    }

    boolean lastJoystickSignal = false;

    @Override
    public void initialize(){
        // So if currently false the signal must go true for at least .2 seconds before being read as a True signal.
        /*if (RobotContainer.joystickOperator.getPovUp()) { // Command.WaitUntilCommand(boolean) or new WaitUntilCommand(boolean)?
            m_climber.advanceClimbState();
            joystickSignal = !joystickSignal;
            System.out.println("Advancing climb state");
        }
        else{
            System.out.println("Button not pressed to advance climb state");
        }
        */
        System.out.println("running");
        m_climber.advanceClimbState();
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("Command Finished");
    }
}