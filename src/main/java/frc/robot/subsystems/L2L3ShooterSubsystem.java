package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
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

public class L2L3ShooterSubsystem extends SubsystemBase {
  
  private CANSparkMax right, left; 
  private SparkPIDController shooterPID;

  public L2L3ShooterSubsystem() { // Constructor 
    right = CANSparkMax(Ports.CAN_L2L3_SHOOTER_RIGHT, MotorType.kBrushless);
    left = CANSparkMax(Ports.CAN_L2L3_SHOOTER_LEFT, MotorType.kBrushless);
  }

  @Override
  private void configureMotors() {
    right.restoreFactoryDefaults(); 
    left.restoreFactoryDefaults();

    right.setInverted(true);

    right.setSmartCurrentLimit();
    left.setSmartCurrentLimit();

    shooterPID.setP(L2L3_SHOOTER_PID); // PID values not in yet 
    shooterPID.setI(L2L3_SHOOTER_PID);
    shooterPID.setD(L2L3_SHOOTER_PID);

    right.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE);
    left.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE);
    right.setOpenLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE);
    left.setOpenLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE);
  }

  public void runMotor(double percent_output){
    left.set(percent_output);
    right.set(percent_output);
  }

  public void stop(){
    left.set(0.0);
    right.set(0.0);
  }
}