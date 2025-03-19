package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;
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
  private SparkClosedLoopController pid_controller;
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

    pid_controller = bottomRight.getClosedLoopController();

    bottomLeftConfig
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(MotionControl.ELEVATOR_CLOSED_LOOP_RAMP_RATE)
      //.inverted(true)
      .openLoopRampRate(MotionControl.ELEVATOR_OPEN_LOOP_RAMP_RATE)
      .follow(bottomRight,true);
    bottomRightConfig
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(MotionControl.ELEVATOR_CLOSED_LOOP_RAMP_RATE)
      .openLoopRampRate(MotionControl.ELEVATOR_OPEN_LOOP_RAMP_RATE)
      .inverted(false)
      .smartCurrentLimit(31);
    bottomRightConfig.closedLoop
      .pid(MotionControl.ELEVATOR_PID.kP, MotionControl.ELEVATOR_PID.kI, MotionControl.ELEVATOR_PID.kD)
      .outputRange(MotionControl.ELEVATOR_MIN_OUTPUT, MotionControl.ELEVATOR_MAX_OUTUT);
    topLeftConfig
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(MotionControl.ELEVATOR_CLOSED_LOOP_RAMP_RATE)
      .openLoopRampRate(MotionControl.ELEVATOR_OPEN_LOOP_RAMP_RATE)
      //.inverted(true)
      .follow(bottomRight,true);
    topRightConfig
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(MotionControl.ELEVATOR_CLOSED_LOOP_RAMP_RATE)
      .openLoopRampRate(MotionControl.ELEVATOR_OPEN_LOOP_RAMP_RATE)
      .inverted(false)
      .follow(bottomRight);

    pid_encoder = bottomRight.getEncoder();

    configureMotors();
  }

  private void configureMotors(){
    bottomLeft.configure(bottomLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRight.configure(bottomRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topLeft.configure(topLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topRight.configure(topRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //bottomLeft is leader *in russian accent with boss music*


    /*
    bottomLeft.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomRight.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topLeft.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topRight.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomLeftConfig.follow(bottomLeft,true);
    bottomRightConfig.follow(bottomRight);
    topLeftConfig.follow(bottomLeft, true);
    topRightConfig.follow(bottomRight);
  

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
     */
}

  public void moveToPosition(double setPoint) {
    if (pid_controller != null) {
      pid_controller.setReference(setPoint, ControlType.kPosition);
    }
  }

  public void zeroElevatorPosition(){

    pid_encoder.setPosition(0);
  }

  public double getPosition() {
    if (pid_encoder == null) {
      return 0;
    }
    return (pid_encoder.getPosition());
  }
}