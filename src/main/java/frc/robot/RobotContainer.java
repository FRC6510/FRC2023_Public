// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
// import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.OuttakeReal;
import frc.robot.subsystems.Drivetrain.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.SwerveDrive;
// import frc.robot.subsystems.ArmPrototype;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ArmReal;
// import frc.robot.subsystems.LEDs;
//import frc.robot.subsystems.OuttakePrototype;
// import frc.robot.subsystems.ArmMotionMagic;
// import frc.robot.subsystems.OuttakePrototype;
import frc.robot.subsystems.Drivetrain.SwerveModule;
import frc.robot.subsystems.Lift.LiftReal;
import frc.robot.commands.auto_test;
import frc.robot.commands.Autonomous.Auto;
import frc.robot.commands.Autonomous.SetLiftAuto;
import frc.robot.commands.Autonomous.Auto.AutoList;
import frc.robot.commands.Drivetrain.ProfiledMoveTo;
import frc.robot.commands.SimplerMainStateMachine.states;
import frc.robot.commands.LiftGo;
import frc.robot.commands.LiftHome;
import frc.robot.commands.SimplerMainStateMachine;
import frc.robot.commands.SwerveDriveStateCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Wrist;
// import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SimplerMainStateMachine;

import java.util.function.BooleanSupplier;

import javax.swing.plaf.basic.BasicBorders.MarginBorder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


  private final XboxController m_joystick = new XboxController(1);

  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;

  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);

  public final Limelight m_limelight = new Limelight();

  public final SwerveDrive swerveDrive = new SwerveDrive();
  public final Wrist m_Wrist = new Wrist();
  public final OuttakeReal m_Outtake = new OuttakeReal();
  public final ArmReal m_Arm = new ArmReal();
  public final LiftReal m_Lift = new LiftReal(); 
  public final Limelight m_Limelight = new Limelight();
  // public final LEDs m_LEDS = new LEDs();
  public SwerveDriveStateCommand m_SwerveDriveStateCommand = new SwerveDriveStateCommand(
    this,
    swerveDrive, 
    m_Limelight, 
    driverController,
    m_Arm);
  
  public Auto m_auto = new Auto(this);
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // = new SendableChooser<>();

    /* TODO: re-introduce constructor
  public SimplerMainStateMachine m_MainStateMachine = new SimplerMainStateMachine(
    m_Lift, 
    m_Arm, 
    m_limelight, 
    m_Wrist, 
    m_Outtake, 
    m_SwerveDriveStateCommand, 
    driverController, 
    operatorController,
    swerveDrive);
    */

  //public final MainStateMachine m_StateMachine = new MainStateMachine(m_Arm, m_limelight, m_Wrist, m_Outtake, null, driverController, operatorController);
  //private final Command driveTestAuto = new auto_drive_test(swerveDrive);
  //private final Command autonomous1 = new frc.robot.commands.Autonomous.autonomous1(swerveDrive);

  //public final PracticeIntake m_PracticeIntake = new PracticeIntake();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController = new CommandXboxController(
    OperatorConstants.kOperatorControllerPort
  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
 
    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    m_chooser.setDefaultOption("Just balance", m_auto.getAuto(AutoList.AUTO_JUST_BALANCE));
    // m_chooser.addOption("blue left", m_auto.getAuto(AutoList.AUTO_BLUE_LEFT));
    // m_chooser.addOption("red right", m_auto.getAuto(AutoList.AUTO_RED_RIGHT));
    m_chooser.addOption("Red right no turn", m_auto.getAuto(AutoList.AUTO_RED_RIGHT_NO_TURN));
    m_chooser.addOption("Blue left no turn", m_auto.getAuto(AutoList.AUTO_BLUE_LEFT_NO_TURN));
    // m_chooser.addOption("Blue left crazy cone", m_auto.getAuto(AutoList.AUTO_CONE_BLUE_LEFT_CRAZYARM));
    // m_chooser.addOption("Red right crazy cone", m_auto.getAuto(AutoList.AUTO_CONE_RED_RIGHT_CRAZYARM));
    // m_chooser.addOption("Red left", m_auto.getAuto(AutoList.AUTO_RED_LEFT));
    // m_chooser.addOption("Blue right", m_auto.getAuto(AutoList.AUTO_BLUE_RIGHT));
    m_chooser.addOption("Just cube", m_auto.getAuto(AutoList.AUTO_JUST_CUBE));
    m_chooser.addOption("Just balance no mobility", m_auto.getAuto(AutoList.AUTO_BALANCE_NO_MOBILITY));




    SmartDashboard.putData(m_chooser);

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    //m_chooser.setDefaultOption("auto 1", autonomous1);
    //Smart.putData(m_chooser);

    //m_driverController.y().onTrue(new ProfiledMoveTo(swerveDrive,new Pose2d(0,1,Rotation2d.fromDegrees(0)),4));
    //m_driverController.b().onTrue(new ProfiledMoveTo(swerveDrive,new Pose2d(0,0,Rotation2d.fromDegrees(0)),4));

    swerveDrive.setDefaultCommand(
      m_SwerveDriveStateCommand
      // new SwerveDriveStateCommand(m_MainStateMachine,
      //   swerveDrive, m_limelight, 
      //   driverController
      // )
    );
    
    m_driverController.y().onTrue(new InstantCommand( () -> 
      swerveDrive.m_odometry.resetPosition(
        SwerveDrive.imu.getRotation2d(),
        new SwerveModulePosition[] {
          swerveDrive.frontLeft.getPosition(),
          swerveDrive.frontRight.getPosition(),
          swerveDrive.backLeft.getPosition(),
          swerveDrive.backRight.getPosition()
        },
        new Pose2d(0,0,new Rotation2d()))));// buttonY.whenPressed(() -> drivetrain.resetGyro(),drivetrain)

        // m_LEDS.changeColour(operatorController);

  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Auto auto = new Auto(this);
    //return auto.getAuto(AutoList.AUTO_JUST_BALANCE);
     return m_chooser.getSelected();
  }
}
