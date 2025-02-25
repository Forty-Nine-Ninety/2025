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

public class L1ShooterSubsystem extends SubsystemBase {
  private SparkMax shooter;

  public L1ShooterSubsystem() {
    outtakeMotor = new SparkMax(Ports.CAN_SHOOTER_SPARKMAX, MotorType.kBrushless);

    pid_L1shooter = tbd.getPIDController();
    pid_encoder = tbd.getEncoder();

    configureMotors();
  }

  private void configureMotors(){
    outtakeMotor.restoreFactoryDefaults();
    outtakeMotor.setSmartCurrentLimit(15);

    // outtakeMotor.setInverted(true);

    pid_L1shooter.setP(MotionControl.L1SHOOTER_tbd_PID.kP);
    pid_L1shooter.setI(MotionControl.L1Shooter_tbd_PID.kI);
    pid_L1shooter.setD(MotionControl.L1Shooter_tbd_PID.kD);

    outtakeMotor.setIdleMode(IdleMode.kBrake);
    // tbd - maybe not

    outtakeMotor.setClosedLoopRampRate(MotionControl.CLOSED_LOOP_RAMP_RATE);
    outtakeMotor.setOpenLoopRampRate(MotionControl.OPEN_LOOP_RAMP_RATE);
  }

  public void runMotor(double percent_output){
    outtakeMotor.set(percent_output);
  }

  public void stop(){
    outtakeMotor.set(0.0);
  }

  /* 
  public void resetMotor(){
  pid_encoder.setPosition(0);
  }
   */
}
