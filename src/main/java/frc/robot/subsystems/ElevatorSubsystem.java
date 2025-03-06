package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.*;
import frc.robot.Constants.Ports.MotionControl;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax bottomLeft, bottomRight, topLeft, topRight;
  private SparkMaxConfig bottomLeftConfig, bottomRightConfig, topLeftConfig, topRightConfig;
  private PIDController pid_elevator;
  private RelativeEncoder pid_encoder;


  public ElevatorSubsystem(){
    bottomLeft = new SparkMax(Ports.CAN_ELEVATOR_BOTTOM_LEFT, MotorType.kBrushless);
    bottomRight = new SparkMax(Ports.CAN_ELEVATOR_BOTTOM_RIGHT, MotorType.kBrushless);
    topLeft = new SparkMax(Ports.CAN_ELEVATOR_TOP_LEFT, MotorType.kBrushless);
    topRight = new SparkMax(Ports.CAN_ELEVATOR_TOP_RIGHT, MotorType.kBrushless);

    bottomLeftConfig = new SparkMaxConfig();
    bottomRightConfig = new SparkMaxConfig();
    topLeftConfig = new SparkMaxConfig();
    topRightConfig = new SparkMaxConfig();

    bottomLeftConfig
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE)
      .openLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE);
    bottomRightConfig
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE)
      .openLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE);
    topLeftConfig
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE)
      .openLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE);
    topRightConfig
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE)
      .openLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE);

    pid_elevator = new PIDController(MotionControl.ELEVATOR_PID.kP,
                                     MotionControl.ELEVATOR_PID.kI,
                                     MotionControl.ELEVATOR_PID.kD);
    configureMotors();

    pid_encoder = bottomLeft.getEncoder();
  }

  private void configureMotors(){
    bottomLeft.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRight.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topLeft.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topRight.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomLeftConfig.follow(bottomLeft,true);
    bottomRightConfig.follow(bottomRight);
    topLeftConfig.follow(bottomLeft, true);
    topRightConfig.follow(bottomRight);
    
    // depends on design from mech
    //bottomLeft.setInverted(true);
    //topLeft.setInverted(true);

    //bottomLeft.setSmartCurrentLimit(60);
    //bottomRight.setSmartCurrentLimit(60);
    //topLeft.setSmartCurrentLimit(60);
    //topRight.setSmartCurrentLimit(60);

    //pid_elevator.setP(MotionControl.ELEVATOR_PID.kP);
    //pid_elevator.setI(MotionControl.ELEVATOR_PID.kI);
    //pid_elevator.setD(MotionControl.ELEVATOR_PID.kD);
    
    //bottomLeft.setIdleMode(IdleMode.kBrake);
    //bottomRight.setIdleMode(IdleMode.kBrake);
    //topLeft.setIdleMode(IdleMode.kBrake);
    //topRight.setIdleMode(IdleMode.kBrake);
    
}

  public void moveToPosition(double setPoint) {
    if (pid_elevator != null) {
      pid_elevator.setSetpoint(setPoint/*, ControlType.kPosition, ClosedLoopSlot.kSlot0, MotionControl.ELEVATOR_FEEDFORWARD*/);
    }
  }

  public void zeroElevatorPosition(){
    if (pid_encoder != null) {
      pid_encoder.setPosition(0);
    }
  }

  public double getPosition() {
    if (pid_encoder == null) {
      return 0;
    }
    return (pid_encoder.getPosition());
  }
}