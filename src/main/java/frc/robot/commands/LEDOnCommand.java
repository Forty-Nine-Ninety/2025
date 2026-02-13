package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlinkinSubsystem;

public class LEDOnCommand extends Command {

    private final BlinkinSubsystem blinkin;

    public LEDOnCommand(BlinkinSubsystem subsystem) {
        this.blinkin = subsystem;
        addRequirements(blinkin);
    }

    @Override
    public void initialize() {
        blinkin.turnOnLED();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;  
    }
}

