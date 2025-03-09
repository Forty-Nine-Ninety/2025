// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.Constants.Ports.DriveSettings;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;

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
  private final VisionAlignLeftCommand m_visionAlignLeftCommand = new VisionAlignLeftCommand(m_drivebase,"Arducam One",joystickOperator);
  private final VisionAlignRightCommand m_visionAlignRightCommand = new VisionAlignRightCommand(m_drivebase,"Arducam One",joystickOperator);

  //Auto
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private SendableChooser<String> m_coralChooser = new SendableChooser<>();
  private SendableChooser<String> m_exitChooser = new SendableChooser<>();
  //private final AutoCommand m_autoCommand = new AutoCommand(m_arm,m_shooter,m_drivebase,"11NBlue");


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    
    // Auto - SmartDashboard
    m_autoChooser.setDefaultOption("Blue 1: Exit", m_drivebase.getAutonomousCommand("1ExitBlue"));
    m_autoChooser.addOption("Blue 3: Exit", m_drivebase.getAutonomousCommand("3ExitBlue"));
    //m_autoChooser.addOption("Red 1: One Coral", new Auto11NRedCommand(m_drivebase,m_arm,m_shooter));
    //m_autoChooser.addOption("Red 2: One Coral", new Auto21NRedCommand(m_drivebase,m_arm,m_shooter));
    //m_autoChooser.addOption("Red 3: One Coral", new Auto31NRedCommand(m_drivebase,m_arm,m_shooter));
    m_autoChooser.addOption("Blue 1: One Coral", new Auto1NBlueCommand(m_drivebase,m_L1shooter,"11CBlue"));
    m_autoChooser.addOption("Blue 2: One Coral", new Auto1NBlueCommand(m_drivebase,m_L1shooter,"21CBlue"));
    m_autoChooser.addOption("Blue 3: One Coral", new Auto1NBlueCommand(m_drivebase,m_L1shooter,"31CBlue"));
    //m_autoChooser.addOption("Red 2 Two Coral", new Auto22NRedCommand(m_drivebase,m_arm,m_shooter,m_intake));
    //m_autoChooser.addOption("Blue 2 Two Coral", new Auto22NBlueCommand(m_drivebase,m_arm,m_shooter,m_intake));

    m_coralChooser.setDefaultOption("Exit", "0");
    m_coralChooser.addOption("1 coral", "1");
    m_coralChooser.addOption("2 coral", "2");
    m_coralChooser.addOption("3 coral", "3");

    m_exitChooser.setDefaultOption("1", "Exit 1");
    m_exitChooser.addOption("2", "Exit 2");
    m_exitChooser.addOption("3", "Exit 3");

    Shuffleboard.getTab("Auto Choose").add("Choose Auto Path", m_autoChooser); //Will need to be updated by Next year if we still use elastic

    SmartDashboard.putData("Coral Chooser",m_coralChooser);
    SmartDashboard.putData("Starting Position",m_exitChooser);

    String m_chosenExit = m_exitChooser.getSelected();
    String m_chosenCoral = m_coralChooser.getSelected();
    switch(m_chosenExit){

      case "Exit 1":
        switch(m_chosenCoral){
          case "0":
            m_autoChooser.setDefaultOption("Blue 1: Exit", m_drivebase.getAutonomousCommand("1ExitBlue"));
            System.out.println("1ExitBlue");
            break;
          
          case "1":
            m_autoChooser.addOption("Blue 1: One Coral", new Auto1NBlueCommand(m_drivebase,m_L1shooter,"11CBlue"));
            System.out.println("11CBlue");
            break;
          
          case"2":
            //m_autoChooser.addOption("Blue 1: Two Coral", new Auto2NBlueCommand(m_drivebase,m_L1shooter,"12CBlue"));
            System.out.println("12CBlue");
            break;
  
          case "3":
            //m_autoChooser.addOption("Blue 1: Three Coral", new Auto3NBlueCommand(m_drivebase,m_L1shooter,"13CBlue"));
            System.out.println("13CBlue");
            break;
        }  

      case "Exit 2":
        switch(m_chosenCoral){
          case "0":
            m_autoChooser.setDefaultOption("Blue 2: Exit", m_drivebase.getAutonomousCommand("2ExitBlue"));
            System.out.println("2ExitBlue");
            break;

          case "1":
            m_autoChooser.addOption("Blue 2: One Coral", new Auto1NBlueCommand(m_drivebase,m_L1shooter,"21CBlue"));
            System.out.println("21CBlue");
            break;

          case"2":
            //m_autoChooser.addOption("Blue 2: Two Coral", new Auto2NBlueCommand(m_drivebase,m_L1shooter,"22CBlue"));
            System.out.println("22CBlue");
            break;

          case "3":
            //m_autoChooser.addOption("Blue 2: Three Coral", new Auto3NBlueCommand(m_drivebase,m_L1shooter,"23CBlue"));
            System.out.println("23CBlue");
            break;
        }  

      case "Exit 3":
        switch(m_chosenCoral){
          case "0":
            m_autoChooser.setDefaultOption("Blue 3: Exit", m_drivebase.getAutonomousCommand("3ExitBlue"));
            System.out.println("3ExitBlue");
            break;

          case "1":
            m_autoChooser.addOption("Blue 3: One Coral", new Auto1NBlueCommand(m_drivebase,m_L1shooter,"31CBlue"));
            System.out.println("31CBlue");
            break;

          case"2":
            //m_autoChooser.addOption("Blue 3: Two Coral", new Auto2NBlueCommand(m_drivebase,m_L1shooter,"32CBlue"));
            System.out.println("32CBlue");
            break;

          case "3":
            //m_autoChooser.addOption("Blue 3: Three Coral", new Auto3NBlueCommand(m_drivebase,m_L1shooter,"33CBlue"));
            System.out.println("33CBlue");
            break;
        }  
    }
    SmartDashboard.updateValues();
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
        //DRIVE CONTROLLER
        m_driveCommand.setSuppliers(
            () -> MathUtil.applyDeadband(joystickDrive.getLeftY(), DriveSettings.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(joystickDrive.getLeftX(), DriveSettings.LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(joystickDrive.getRightX(), DriveSettings.RIGHT_X_DEADBAND)
        );

        joystickDrive.a().onTrue(Commands.runOnce(m_drivebase::zeroGyro));

        //OPERATOR CONTROLLER
        m_elevatorManualCommand.setSuppliers(
          () -> MathUtil.applyDeadband(DriveUtil.powCopySign(joystickOperator.getLeftY(),1),DriveSettings.ARM_DEADBAND)
        );

        joystickOperator.a().onTrue(m_elevatorL1Command);
        joystickOperator.b().onTrue(m_elevatorL2Command);
        joystickOperator.y().onTrue(m_elevatorL3Command);
        joystickOperator.x().onTrue(m_elevatorNeutralCommand);
        joystickOperator.povUp().onTrue(m_elevatorClimbCommand);
        joystickOperator.povDown().onTrue(m_intakeElevatorCommand);
        joystickOperator.povLeft().onTrue(m_visionAlignLeftCommand);
        joystickOperator.povRight().onTrue(m_visionAlignRightCommand);

        joystickOperator.leftBumper().onTrue(m_outtakeL1Command);
        joystickOperator.rightBumper().onTrue(m_outtakeL2L3Command);

        joystickOperator.rightStick().onTrue(m_elevatorManualCommand);
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
  // public Command getAutonomousCommand() {
    //return m_autoChooser.getSelected();
  // }

  public void setMotorBrake(boolean brake)
  {
    m_drivebase.setMotorBrake(brake);
  }
}