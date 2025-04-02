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

public class L1ShooterSubsystem extends SubsystemBase {

  private SparkMax outtakeMotor;
  //private SparkClosedLoopController pid_L1shooter;
  //private RelativeEncoder pid_encoder; 

  public L1ShooterSubsystem() {
    outtakeMotor = new SparkMax(Ports.CAN_L1SHOOTER, MotorType.kBrushless);
    SparkMaxConfig outtakeMotorConfig = new SparkMaxConfig(); 

    outtakeMotorConfig
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(MotionControl.ELEVATOR_CLOSED_LOOP_RAMP_RATE)
      .openLoopRampRate(MotionControl.ELEVATOR_OPEN_LOOP_RAMP_RATE)
      .smartCurrentLimit(15); 

    configureMotors();
  }

  private void configureMotors(){
    outtakeMotor.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // outtakeMotor.setInverted(true);
  }

  public void runMotor(double percent_output){
    outtakeMotor.set(percent_output);
  }
}
