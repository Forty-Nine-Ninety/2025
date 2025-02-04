package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {

  private CANSparkMax bottomLeft, bottomRight, topLeft, topRight;
  private SparkPIDController pid_elevator;
  private RelativeEncoder pid_encoder;


  public ElevatorSubsystem(){
    bottomLeft = new CANSparkMax(Ports.CAN_ELEVATOR_BOTTOM_LEFT, MotorType.kBrushless);
    bottomRight = new CANSparkMax(Ports.CAN_ELEVATOR_BOTTOM_RIGHT, MotorType.kBrushless);
    topLeft = new CANSparkMax(Ports.CAN_ELEVATOR_TOP_LEFT, MotorType.kBrushless);
    topRight = new CANSparkMax(Ports.CAN_ELEVATOR_TOP_RIGHT, MotorType.kBrushless);
    
    pid_elevator = tbd.getPIDController();
    pid_encoder = tbd.getEncoder();
    configureMotors();
  }

  private void configureMotors(){
    bottomLeft.restoreFactoryDefaults();
    bottomRight.restoreFactoryDefaults();
    topLeft.restoreFactoryDefaults();
    topRight.restoreFactoryDefaults();

    // depends on design from mech
    // bottomLeft.setInverted(true);
    // topLeft.setInverted(true);

    bottomLeft.setSmartCurrentLimit(60);
    bottomRight.setSmartCurrentLimit(60);
    topLeft.setSmartCurrentLimit(60);
    topRight.setSmartCurrentLimit(60);

    pid_elevator.setP(MotionControl.ELEVATOR_tbd_PID.kP);
    pid_elevator.setI(MotionControl.ELEVATOR_tbd_PID.kI);
    pid_elevator.setD(MotionControl.ELEVATOR_tbd_PID.kD);
    
    bottomLeft.setIdleMode(IdleMode.kBrake);
    bottomRight.setIdleMode(IdleMode.kBrake);
    topLeft.setIdleMode(IdleMode.kBrake);
    topRight.setIdleMode(IdleMode.kBrake);

    // all ramp rates are currently 1 (one)
    topLeft.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE); 
    topLeft.setOpenLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE); 

    topRight.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE); 
    topRight.setOpenLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE); 

    bottomLeft.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE); 
    bottomLeft.setOpenLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE); 

    bottomRight.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE); 
    bottomRight.setOpenLoopRampRate(MwotionControl.OPEN_LOOP_RAMP_RATE); 
    
}

  public void moveToPosition(double setPoint) {
    pid_elevator.setReference(setPoint, CANSparkBase.ControlType.kPosition, 0, MotionControl.ELEVATOR_FEEDFORWARD);
  }

  public void resetElevatorPosition(){
    pid_encoder.setPosition(0);
  }

  public double getPosition() {
    return (pid_encoder.getPosition());
  }
}