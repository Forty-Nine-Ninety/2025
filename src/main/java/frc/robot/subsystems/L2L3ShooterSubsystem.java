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

public class L2L3ShooterSubsystem extends SubsystemBase {
  
  private CANSparkMax right, left; 
  private SparkPIDController shooterPID;
  private final DigitalInput m_breakbeam;

  public L2L3ShooterSubsystem() { // Constructor 
    right = CANSparkMax(Ports.CAN_L2L3_SHOOTER_RIGHT, MotorType.kBrushless);
    left = CANSparkMax(Ports.CAN_L2L3_SHOOTER_LEFT, MotorType.kBrushless);
    m_breakbeam = new DigitalInput(Ports.PORT_DIO_BREAK_BEAM);
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

  public boolean breakBeam(){
    return !m_breakbeam.get();
  }
}