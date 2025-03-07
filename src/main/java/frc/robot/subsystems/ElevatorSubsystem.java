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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.*;
import frc.robot.Constants.Ports.MotionControl;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax bottomLeft, bottomRight, topLeft, topRight;
  private SparkMaxConfig bottomLeftConfig, bottomRightConfig, topLeftConfig, topRightConfig;
  private PIDController pid_controller;
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
    bottomLeftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(1.0, 0.0, 0.0);
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


    pid_encoder = bottomLeft.getEncoder();

    configureMotors();
  }

  private void configureMotors(){
    bottomLeft.configure(bottomLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRight.configure(bottomRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topLeft.configure(topLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topRight.configure(topRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //bottomLeft is leader
    bottomRightConfig.follow(bottomLeft,true);
    topLeftConfig.follow(bottomLeft);
    topRightConfig.follow(bottomLeft,true);

    /*
    bottomLeft.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRight.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topLeft.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topRight.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomLeftConfig.follow(bottomLeft,true);
    bottomRightConfig.follow(bottomRight);
    topLeftConfig.follow(bottomLeft, true);
    topRightConfig.follow(bottomRight);
    */

    //pid_controller.setP(MotionControl.ELEVATOR_PID.kP);
    //pid_controller.setI(MotionControl.ELEVATOR_PID.kI);
    //pid_controller.setD(MotionControl.ELEVATOR_PID.kD);

    bottomLeftConfig.smartCurrentLimit(60);
    bottomRightConfig.smartCurrentLimit(60);
    topLeftConfig.smartCurrentLimit(60);
    topRightConfig.smartCurrentLimit(60);
    
    bottomLeftConfig.idleMode(IdleMode.kBrake);
    bottomRightConfig.idleMode(IdleMode.kBrake);
    topLeftConfig.idleMode(IdleMode.kBrake);
    topRightConfig.idleMode(IdleMode.kBrake);
    
}

  public void moveToPosition(double setPoint) {
    System.out.println("moveToPosition reached");
    if (pid_controller != null) {
      for(int i=0;i<20;i++){
        System.out.println(pid_controller.getSetpoint());
      }
      pid_controller.setSetpoint(setPoint/*, ControlType.kPosition, ClosedLoopSlot.kSlot0, MotionControl.ELEVATOR_FEEDFORWARD*/);
    }
    System.out.println("moveToPosition executed");
  }

  public void zeroElevatorPosition(){
    System.out.println("zeroElevatorPosition reached");
    if (pid_encoder != null) {
      pid_encoder.setPosition(0);
    }
    System.out.println("zeroElevatorPosition executed");
  }

  public double getPosition() {
    System.out.println("getPosition reached");
    if (pid_encoder == null) {
      System.out.println("pid_encoder==null");
      return 0;
    }
    return (pid_encoder.getPosition());
  }
}