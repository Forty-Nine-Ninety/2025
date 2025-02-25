// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.Constants.Ports.DriveSettings;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  

  CommandXboxController joystickDrive = new CommandXboxController(Ports.PORT_JOYSTICK_DRIVE);
  CommandXboxController joystickOperator = new CommandXboxController(Ports.PORT_JOYSTICK_OPERATOR);

  // The robot's subsystems and commands are defined here...
  //Subsystems
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final L1ShooterSubsystem m_L1shooter = new L1ShooterSubsystem();
  private final L2L3ShooterSubsystem m_L2L3shooter = new L2L3ShooterSubsystem();
  //Commands
  private final DriveCommand m_driveCommand = new DriveCommand(m_drivebase);
  private final ElevatorClimbCommand m_elevatorClimbCommand = new ElevatorClimbCommand(m_elevator);
  private final ElevatorL1Command m_elevatorL1Command = new ElevatorL1Command(m_elevator);
  private final ElevatorL2Command m_elevatorL2Command = new ElevatorL2Command(m_elevator);
  private final ElevatorL3Command m_elevatorL3Command = new ElevatorL3Command(m_elevator);
  private final ElevatorManualCommand m_elevatorManualCommand = new ElevatorManualCommand(m_elevator);
  private final ElevatorNeutralCommand m_elevatorNeutralCommand = new ElevatorNeutralCommand(m_elevator);
  private final IntakeElevatorCommand m_intakeElevatorCommand = new IntakeElevatorCommand(m_elevator, m_L2L3shooter);
  private final OuttakeL1Command m_outtakeL1Command = new OuttakeL1Command(m_L1shooter);
  private final OuttakeL2L3Command m_outtakeL2L3Command = new OuttakeL2L3Command(m_L2L3shooter);
  //Auto
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  //private final AutoCommand m_autoCommand = new AutoCommand(m_arm,m_shooter,m_drivebase,"11NBlue");


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //Auto SmartDashboard sendable chooser
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings()
    {
        //XBOX
        m_driveCommand.setSuppliers(
            () -> MathUtil.applyDeadband(joystickDrive.getLeftY(), DriveSettings.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(joystickDrive.getLeftX(), DriveSettings.LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(joystickDrive.getRightX(), DriveSettings.RIGHT_X_DEADBAND)
        );

        m_elevatorManualCommand.setSuppliers(
          () -> MathUtil.applyDeadband(DriveUtil.powCopySign(joystickOperator.getLeftY(),1),DriveSettings.ARM_DEADBAND)
        );
        
  }
  public void setTeleopDefaultCommands()
    {
        CommandScheduler.getInstance().setDefaultCommand(m_drivebase, m_driveCommand);
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return m_autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
      m_drivebase.setMotorBrake(brake);
  }
}
