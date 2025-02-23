package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ElevatorManualCommand extends Command{
    private final ElevatorSubsystem m_elevator;
    private double multiplier = 36.0;
    private DoubleSupplier m_joystickSupplier;

    public ElevatorNeutralCommand(ElevatorSubsystem elevator,DoubleSupplier joystickSupplier){
        m_elevator = elevator;
        addRequirements(elevator);
    }

    public void setSuppliers(DoubleSupplier joystickSupplier){
        m_joystickSupplier = joystickSupplier;
    }

    @Override
    public void execute(){
        double position = m_joystickSupplier.getAsDouble;
        double currentRotation = m_elevator.getPosition();
        m_elevator.moveToPosition(currentRotation+(position*multiplier));
    }
}