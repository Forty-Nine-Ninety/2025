package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Ports.MotionControl;

public class ClimberSubsystem extends SubsystemBase {
    private SparkMax stingerMotor,clipMotor,trapdoorMotor;
    private SparkClosedLoopController stingerController,clipController,trapdoorController;
    private ElevatorSubsystem m_elevator;

    private enum ClimbState {
        NOT_CLIMBING,
        TRAPDOOR_LIFTED,
        ELEVATOR_CLIMB_POS,
        STINGER_DEPLOYED,
        STINGER_CAGE_LIFTED,
        ELEVATOR_TO_ZERO,
        CLIPS_ENGAGED
    };

    private ClimbState m_climbState = ClimbState.NOT_CLIMBING;

    public ClimberSubsystem(ElevatorSubsystem elevator) {
        m_elevator = elevator;
        stingerMotor = new SparkMax(Ports.CAN_STINGER_ROTATION, MotorType.kBrushless);
        clipMotor = new SparkMax(Ports.CAN_ELEVATOR_CLIPS, MotorType.kBrushless);
        trapdoorMotor = new SparkMax(Ports.CAN_TRAPDOOR, MotorType.kBrushless);

        SparkMaxConfig stingerConfig = new SparkMaxConfig();
        SparkMaxConfig clipConfig = new SparkMaxConfig();
        SparkMaxConfig trapdoorConfig = new SparkMaxConfig();

        stingerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotionControl.STINGER_AMP_LIMIT)
            .inverted(true)
            .closedLoop
                .pid(MotionControl.STINGER_PID.kP, MotionControl.STINGER_PID.kI, MotionControl.STINGER_PID.kD)
            ;

        clipConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotionControl.CLIPS_AMP_LIMIT)
            .closedLoop
                .pid(MotionControl.CLIPS_PID.kP, MotionControl.CLIPS_PID.kI, MotionControl.CLIPS_PID.kD)
            //.inverted(true)
            ;

        trapdoorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotionControl.TRAPDOOR_AMP_LIMIT)
            .inverted(true)
            .closedLoop
                .pid(MotionControl.TRAPDOOR_PID.kP, MotionControl.TRAPDOOR_PID.kI, MotionControl.TRAPDOOR_PID.kD)
            ;

        stingerController = stingerMotor.getClosedLoopController();
        clipController = clipMotor.getClosedLoopController();
        trapdoorController = trapdoorMotor.getClosedLoopController();

        stingerMotor.configure(stingerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        clipMotor.configure(clipConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        trapdoorMotor.configure(trapdoorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }



    public void liftTrapdoor(){
        trapdoorController.setReference(MotionControl.TRADPDOOR_LIFTED_POSITION, ControlType.kPosition);
    }

    public void setElevator(){
        m_elevator.moveToPosition(MotionControl.ELEVATOR_STINGER_POSITION);
    }

    public void deployStinger(){
        stingerController.setReference(MotionControl.STINGER_DEPLOYED_POSITION, ControlType.kPosition);
    }

    public void liftCageUp(){
        stingerController.setReference(MotionControl.STINGER_CAGEUP_POSITION, ControlType.kPosition);
    }

    public void zeroElevator(){
        m_elevator.moveToPosition(MotionControl.ELEVATOR_ZERO_POSITION);
    }

    public void clip(){
        clipController.setReference(MotionControl.CLIPS_ENGAGED_POSITION, ControlType.kPosition);
    }

    public void advanceClimbState() {
        switch (m_climbState) {
            case NOT_CLIMBING:
                // lift trapdoor, advance state to trapdoor-lifted
                System.out.println("Step 1 Initiated");
                trapdoorController.setReference(MotionControl.TRADPDOOR_LIFTED_POSITION, ControlType.kPosition);
                m_climbState = ClimbState.TRAPDOOR_LIFTED;
                System.out.println(m_climbState);
                break;
            case TRAPDOOR_LIFTED:
                // move elevator to CLIMB_POS and advance state
                m_elevator.moveToPosition(MotionControl.ELEVATOR_STINGER_POSITION);
                m_climbState = ClimbState.ELEVATOR_CLIMB_POS;
                System.out.println(m_climbState);
                break;
            case ELEVATOR_CLIMB_POS:
                // deploy stinger and advance state
                System.out.println("Step 3 Initiated");
                stingerController.setReference(MotionControl.STINGER_DEPLOYED_POSITION, ControlType.kPosition);
                m_climbState = ClimbState.STINGER_DEPLOYED;
                System.out.println(m_climbState);
                break;
            case STINGER_DEPLOYED:
                // move stinger to CAGE_LIFTED position and advance state
                System.out.println("Step 4 Initiated");
                stingerController.setReference(MotionControl.STINGER_CAGEUP_POSITION, ControlType.kPosition);
                m_climbState = ClimbState.STINGER_CAGE_LIFTED;
                System.out.println(m_climbState);
                break;
            case STINGER_CAGE_LIFTED:
                // move elevator to zero and advance state
                System.out.println("Step 5 Initiated");
                m_elevator.moveToPosition(MotionControl.ELEVATOR_ZERO_POSITION);
                m_climbState = ClimbState.ELEVATOR_TO_ZERO;
                System.out.println(m_climbState);
                break;
            case ELEVATOR_TO_ZERO:
                // engage clipse and advance state
                System.out.println("Step 6 Initiated");
                clipController.setReference(MotionControl.CLIPS_ENGAGED_POSITION, ControlType.kPosition);
                m_climbState = ClimbState.CLIPS_ENGAGED;
                System.out.println(m_climbState);
                break;
            case CLIPS_ENGAGED:
                System.out.println("Clips already engaged! Climbing should be done");
                break;
        }
    }

    public void reverseClimbState() {
        switch (m_climbState) {
            case NOT_CLIMBING:
                System.out.println("Not in climbing mode!");
                break;
            case TRAPDOOR_LIFTED:
                // lower trapdoor and update state
                trapdoorController.setReference(MotionControl.TRADPDOOR_LOWERED_POSITION, ControlType.kPosition);
                m_climbState = ClimbState.NOT_CLIMBING;
                break;
            case ELEVATOR_CLIMB_POS:
                // update state
                m_climbState = ClimbState.TRAPDOOR_LIFTED;
                break;
            case STINGER_DEPLOYED:
                // move stinger to CAGE_LIFTED position and update state
                stingerController.setReference(MotionControl.STINGER_CAGEUP_POSITION, ControlType.kPosition);
                m_climbState = ClimbState.ELEVATOR_CLIMB_POS;
                break;
            case STINGER_CAGE_LIFTED:
                // move stinger back to engage position and update state
                stingerController.setReference(MotionControl.STINGER_DEPLOYED_POSITION, ControlType.kPosition);
                m_climbState = ClimbState.STINGER_DEPLOYED;
                break;
            case ELEVATOR_TO_ZERO:
                // Move elevator back to stinger position and update state
                m_elevator.moveToPosition(MotionControl.ELEVATOR_STINGER_POSITION);
                m_climbState = ClimbState.STINGER_CAGE_LIFTED;
                break;
            case CLIPS_ENGAGED:
                // disengage clips and update state
                clipController.setReference(MotionControl.CLIPS_DISENGAGED_POSITION, ControlType.kPosition);
                m_climbState = ClimbState.ELEVATOR_TO_ZERO;
                break;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // Publish the solenoid state to telemetry.
        builder.addStringProperty("Climb State", () -> m_climbState.toString(), null);
    }

}
