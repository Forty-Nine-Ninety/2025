package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;

import frc.robot.Constants.*;
import frc.robot.Constants.Ports.MotionControl;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;


public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax bottomLeft, bottomRight, topLeft, topRight;
  private SparkClosedLoopController pid_elevator;
  private RelativeEncoder pid_encoder;


  public ElevatorSubsystem(){
    bottomLeft = new SparkMax(Ports.CAN_ELEVATOR_BOTTOM_RIGHT, MotorType.kBrushless);
    bottomRight = new SparkMax(Ports.CAN_ELEVATOR_BOTTOM_RIGHT, MotorType.kBrushless);
    topLeft = new SparkMax(Ports.CAN_ELEVATOR_TOP_LEFT, MotorType.kBrushless);
    topRight = new SparkMax(Ports.CAN_ELEVATOR_TOP_RIGHT, MotorType.kBrushless);

    SparkMaxConfig bottomLeftConfig = new SparkMaxConfig();
    SparkMaxConfig bottomRightConfig = new SparkMaxConfig();
    SparkMaxConfig topLeftConfig = new SparkMaxConfig();
    SparkMaxConfig topRightConfig = new SparkMaxConfig();

<<<<<<< Updated upstream
    bottomLeftConfig.idleMode(IdleMode.kBrake);
    bottomRightConfig.idleMode(IdleMode.kBrake);
    topLeftConfig.idleMode(IdleMode.kBrake);
    topRightConfig.idleMode(IdleMode.kBrake);
=======
    bottomLeftConfig
      .idleMode(IdleMode.kBrake);
    bottomLeftConfig.closedLoop
      .pid(0, 0, 0);
    bottomRightConfig
      .idleMode(IdleMode.kBrake);
    topLeftConfig
      .idleMode(IdleMode.kBrake);
    topRightConfig
      .idleMode(IdleMode.kBrake);
>>>>>>> Stashed changes
      
    //pid_elevator = tbd.getPIDController();
    //pid_encoder = tbd.getEncoder();
    configureMotors();
    
  }

  private void configureMotors(){
    bottomLeft.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRight.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topLeft.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topRight.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // depends on design from mech
    // bottomLeft.setInverted(true);
    // topLeft.setInverted(true);

    //bottomLeft.setSmartCurrentLimit(60);
    //bottomRight.setSmartCurrentLimit(60);
    //topLeft.setSmartCurrentLimit(60);
    //topRight.setSmartCurrentLimit(60);

    //pid_elevator.setP(MotionControl.ELEVATOR_tbd_PID.kP);
    //pid_elevator.setI(MotionControl.ELEVATOR_tbd_PID.kI);
    //pid_elevator.setD(MotionControl.ELEVATOR_tbd_PID.kD);
    
    //bottomLeft.setIdleMode(IdleMode.kBrake);
    //bottomRight.setIdleMode(IdleMode.kBrake);
    //topLeft.setIdleMode(IdleMode.kBrake);
    //topRight.setIdleMode(IdleMode.kBrake);


    // all ramp rates are currently 1 (one)
    topLeft.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE); 
    topLeft.setOpenLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE); 

    topRight.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE); 
    topRight.setOpenLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE); 

    bottomLeft.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE); 
    bottomLeft.setOpenLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE); 

    bottomRight.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE); 
    bottomRight.setOpenLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE); 
    
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