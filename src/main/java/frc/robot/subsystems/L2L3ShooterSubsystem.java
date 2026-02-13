package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.*;
import frc.robot.Constants.Ports.MotionControl;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class L2L3ShooterSubsystem extends SubsystemBase {
  
  private SparkMax right, left;
  private final DigitalInput m_breakbeam;

  public L2L3ShooterSubsystem() { 
    right = new SparkMax(Ports.CAN_L2L3SHOOTER_RIGHT, MotorType.kBrushless);
    left = new SparkMax(Ports.CAN_L2L3SHOOTER_LEFT, MotorType.kBrushless);
    m_breakbeam = new DigitalInput(Ports.PORT_DIO_BREAK_BEAM);

    SparkMaxConfig rightconfig = new SparkMaxConfig();
    SparkMaxConfig leftconfig = new SparkMaxConfig();

    rightconfig
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(MotionControl.ELEVATOR_CLOSED_LOOP_RAMP_RATE)
      .openLoopRampRate(MotionControl.ELEVATOR_OPEN_LOOP_RAMP_RATE);
    leftconfig
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(MotionControl.ELEVATOR_CLOSED_LOOP_RAMP_RATE)
      .openLoopRampRate(MotionControl.ELEVATOR_OPEN_LOOP_RAMP_RATE);

      configureMotors();
  }

  private void configureMotors() {
    right.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    left.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runMotor(double percent_output){
    left.set(-percent_output);
    right.set(percent_output);
  }

  public boolean breakBeam(){
    return !m_breakbeam.get();
  }
}