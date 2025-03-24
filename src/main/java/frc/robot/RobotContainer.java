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

  public Command m_chosenAuto = null;
    CommandXboxController joystickDrive = new CommandXboxController(Ports.PORT_JOYSTICK_DRIVE);
    CommandXboxController joystickOperator = new CommandXboxController(Ports.PORT_JOYSTICK_OPERATOR);
  
    // The robot's subsystems and commands are defined here...
    //Subsystems
    private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
   // private final ClimberSubsystem m_climber = new ClimberSubsystem(m_elevator);
    private final L1ShooterSubsystem m_L1shooter = new L1ShooterSubsystem();
    private final L2L3ShooterSubsystem m_L2L3shooter = new L2L3ShooterSubsystem();
    //Commands
    private final DriveCommand m_driveCommand = new DriveCommand(m_drivebase);
  //  private final ElevatorClimbCommand m_elevatorClimbCommand = new ElevatorClimbCommand(m_climber);
    //private final ElevatorUnclimbCommand m_elevatorUnclimbCommand = new ElevatorUnclimbCommand(m_climber);
    private final ElevatorL1Command m_elevatorL1Command = new ElevatorL1Command(m_elevator);
    private final ElevatorL2Command m_elevatorL2Command = new ElevatorL2Command(m_elevator);
    private final ElevatorL3Command m_elevatorL3Command = new ElevatorL3Command(m_elevator);
    private final ElevatorManualCommand m_elevatorManualCommand = new ElevatorManualCommand(m_elevator);
    private final ElevatorNeutralCommand m_elevatorNeutralCommand = new ElevatorNeutralCommand(m_elevator);
    private final IntakeElevatorCommand m_intakeElevatorCommand = new IntakeElevatorCommand(m_elevator, m_L2L3shooter);
    private final OuttakeL1Command m_outtakeL1Command = new OuttakeL1Command(m_L1shooter);
    private final OuttakeL2L3Command m_outtakeL2L3Command = new OuttakeL2L3Command(m_L2L3shooter);
    private final VisionAlignLeftCommand m_visionAlignLeftCommand = new VisionAlignLeftCommand(m_drivebase,"scamcam",joystickOperator);
    private final VisionAlignRightCommand m_visionAlignRightCommand = new VisionAlignRightCommand(m_drivebase,"scamcam",joystickOperator);
    
  
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
      NamedCommands.registerCommand("L1Shooter",new AutoL1ShooterCommand(m_L1shooter));
  
      m_autoChooser.setDefaultOption("Blue 1: Exit", m_drivebase.getAutonomousCommand("1ExitBlue"));
      m_autoChooser.addOption("Blue 3: Exit", m_drivebase.getAutonomousCommand("3ExitBlue"));
      m_autoChooser.addOption("Blue 1: One Coral", new Auto1NBlueCommand(m_drivebase,m_L1shooter,"11CBlue"));
      m_autoChooser.addOption("Blue 2: One Coral", new Auto1NBlueCommand(m_drivebase,m_L1shooter,"21CBlue"));
      m_autoChooser.addOption("Blue 3: One Coral", new Auto1NBlueCommand(m_drivebase,m_L1shooter,"31CBlue"));
      
      Shuffleboard.getTab("Auto Choose").add("Choose Auto Path", m_autoChooser);
      
      m_coralChooser.setDefaultOption("Exit","0");
      m_coralChooser.addOption("1 Coral","1");
      //m_coralChooser.addOption("2 Coral","2");
      //m_coralChooser.addOption("3 Coral","3");
  
      m_exitChooser.setDefaultOption("Blue Exit 1","1");
      m_exitChooser.addOption("Exit 2","2");
      m_exitChooser.addOption("Blue Exit 3","3");
      String m_chosenExit = m_exitChooser.getSelected();
      String m_chosenCoral = m_coralChooser.getSelected();
      //Command m_chosenAuto;

    switch(m_chosenExit){
      case "1":
        switch(m_chosenCoral){
          case "0":
            m_chosenAuto = m_drivebase.getAutonomousCommand("1ExitBlue");
          case "1":
            m_chosenAuto = m_drivebase.getAutonomousCommand("11CBlue");
        }
      case "2":
        switch(m_chosenCoral){
          case "1":
            m_chosenAuto = m_drivebase.getAutonomousCommand("21CBlue");
        }  
      case "3":
        switch(m_chosenCoral){
          case "0":
            m_chosenAuto = m_drivebase.getAutonomousCommand("3ExitBlue");
          case "1":
            m_chosenAuto = m_drivebase.getAutonomousCommand("31CBlue");
            
        }
      default:
        m_chosenAuto = m_drivebase.getAutonomousCommand("1ExitBlue");
    }
    System.out.println(m_chosenAuto);
    //SmartDashboard.updateValues();
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
            () -> MathUtil.applyDeadband(-joystickDrive.getLeftY(), DriveSettings.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-joystickDrive.getLeftX(), DriveSettings.LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(-joystickDrive.getRightX(), DriveSettings.RIGHT_X_DEADBAND)
        );

        joystickDrive.a().onTrue(Commands.runOnce(m_drivebase::zeroGyro));
        joystickDrive.leftBumper().toggleOnTrue(m_outtakeL1Command);
        joystickDrive.rightBumper().toggleOnTrue(m_outtakeL2L3Command);

        //OPERATOR CONTROLLER
        m_elevatorManualCommand.setSuppliers(
          () -> MathUtil.applyDeadband(DriveUtil.powCopySign(joystickOperator.getRightY(),1),DriveSettings.ARM_DEADBAND)
        );

        joystickOperator.a().onTrue(m_elevatorL1Command);
        joystickOperator.b().onTrue(m_elevatorL2Command);
        joystickOperator.y().onTrue(m_elevatorL3Command);
        joystickOperator.x().onTrue(m_elevatorNeutralCommand);
       // joystickOperator.povUp().toggleOnTrue(m_elevatorClimbCommand);
        joystickOperator.povDown().onTrue(m_intakeElevatorCommand);
        joystickOperator.povLeft().toggleOnTrue(m_visionAlignLeftCommand);
        joystickOperator.povRight().toggleOnTrue(m_visionAlignRightCommand);

        joystickOperator.leftBumper().toggleOnTrue(m_outtakeL1Command);
        joystickOperator.rightBumper().toggleOnTrue(m_outtakeL2L3Command);

        joystickOperator.rightStick().onTrue(m_elevatorManualCommand);

        //joystickDrive.button(8).onTrue(m_elevatorClimbCommand);
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
    System.out.println("getAutonomousCommand");
    return m_autoChooser.getSelected();
    //return m_drivebase.getAutonomousCommand("1ExitBlue");
    //return new Auto1NBlueCommand(m_drivebase,m_L1shooter,"11CBlue");
  }

  public void setMotorBrake(boolean brake)
  {
    m_drivebase.setMotorBrake(brake);
  }
}