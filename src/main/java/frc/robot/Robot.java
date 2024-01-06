// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autonomous.Auto;
import frc.robot.commands.Autonomous.Auto.AutoList;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

import frc.robot.subsystems.Lift.LiftReal;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.OuttakeReal;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ArmReal;
// import frc.robot.subsystems.LEDs;
import frc.robot.commands.SimplerMainStateMachine;
import frc.robot.commands.SwerveDriveStateCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private SimplerMainStateMachine m_MainStateMachine;

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // SimplerMainStateMachine m_StateMachine;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.


    m_robotContainer = new RobotContainer();
    m_MainStateMachine = new SimplerMainStateMachine(
      m_robotContainer.m_Lift, 
      m_robotContainer.m_Arm, 
      m_robotContainer.m_limelight, 
      m_robotContainer.m_Wrist, 
      m_robotContainer.m_Outtake, 
      m_robotContainer.m_SwerveDriveStateCommand, 
      m_robotContainer.driverController, 
      m_robotContainer.operatorController,
      m_robotContainer.swerveDrive
      // m_robotContainer.m_LEDS
      );

      m_robotContainer.swerveDrive.resetGyro();


    m_robotContainer.camera1 = CameraServer.startAutomaticCapture(0);
    // m_robotContainer.camera2 = CameraServer.startAutomaticCapture(1);
    m_robotContainer.server = CameraServer.getServer();
  
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    

    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_robotContainer.swerveDrive.m_odometry.resetPosition(
    //   SwerveDrive.imu.getRotation2d(),
    //   new SwerveModulePosition[] {
    //     m_robotContainer.swerveDrive.frontLeft.getPosition(),
    //     m_robotContainer.swerveDrive.frontRight.getPosition(),
    //     m_robotContainer.swerveDrive.backLeft.getPosition(),
    //     m_robotContainer.swerveDrive.backRight.getPosition()
    //   },
    //   new Pose2d(0,0,new Rotation2d())
    // );

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
   
    if (m_autonomousCommand != null) {
      // m_autonomousCommand.cancel();
    }

    // m_MainStateMachine.resetStates();

    // m_robotContainer.swerveDrive.m_odometry.resetPosition(
    //   SwerveDrive.imu.getRotation2d(),
    //   new SwerveModulePosition[] {
    //     m_robotContainer.swerveDrive.frontLeft.getPosition(),
    //     m_robotContainer.swerveDrive.frontRight.getPosition(),
    //     m_robotContainer.swerveDrive.backLeft.getPosition(),
    //     m_robotContainer.swerveDrive.backRight.getPosition()
    // }
    // , new Pose2d(0,0,new Rotation2d()));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {    // m_robotContainer.m_Outtake.spinIntake(1, m_robotContainer.operatorController);
    m_MainStateMachine.execute();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
