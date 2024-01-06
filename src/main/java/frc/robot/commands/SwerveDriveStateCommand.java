// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathShared;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmReal;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.SwerveDrive;
import frc.robot.commands.Autonomous.SetLiftAuto;
import frc.robot.commands.SimplerMainStateMachine.scoringPosition;
import frc.robot.RobotContainer;

import frc.robot.commands.SimplerMainStateMachine;

public class SwerveDriveStateCommand extends CommandBase {
  private SwerveDrive m_SwerveDrive;
  private Limelight m_Limelight;
  private XboxController m_driverController;
  private ArmReal m_Arm;

  private final double yMultiplier = 0.025;//0.059;
  private final double xMultiplier = 0.35;
  private double taMultiplier = 1 / 0.1;// 0.12; //1 / 0.15;//TODO check if right 

  private boolean flag = false;
  private double targetAngle = 0;


  boolean driverInput;
  boolean driveLineup;
  boolean driveBalance;
  boolean linedUp;

  public double throttle;

  // public double throttle;


  // public boolean driverInput = (Math.abs(m_driverController.getLeftX()) > 0.1 || Math.abs(m_driverController.getLeftY()) > 0.1) || Math.abs(m_driverController.getRightX()) > 0.1;
  // public boolean linedUp = (deadZone(m_Limelight.getY() * yMultiplier) == 0.0 && deadZone(m_Limelight.getX() * -1) == 0 && m_Limelight.getValid());
  public enum states{
    LINEUP,
    LINEUPSUBSTATION,
    MANUAL,
    BALANCE,
    BALANCED;
    // CAMERA;
  }

  public states state = states.MANUAL;

  
  enum driverChoice{
    RESET,
    NONE,
    CUBE,
    CONE;
  }
  static driverChoice m_driverChoice = driverChoice.NONE;


  /** Creates a new findAprilTag. */
  public SwerveDriveStateCommand(RobotContainer m_RobotContainer, SwerveDrive m_SwerveDrive, Limelight m_Limelight, XboxController m_driverController, ArmReal m_Arm) {
    this.m_Limelight = m_Limelight;
    this.m_SwerveDrive = m_SwerveDrive;
    this.m_driverController = m_driverController;
    this.m_Arm = m_Arm;
    // this.m_MainStateMachine = m
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_SwerveDrive, this.m_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    filterTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driverInput = (Math.abs(m_driverController.getLeftX()) > 0.1 || Math.abs(m_driverController.getLeftY()) > 0.1 || Math.abs(m_driverController.getRightX()) > 0.1);
    linedUp = (deadZone(m_Limelight.getX() * yMultiplier) == 0.0 && deadZone(m_Limelight.getY() * xMultiplier) == 0.0 && m_Limelight.getValid());

    //driveLineup = m_driverController.getYButton();
    driveBalance = m_driverController.getXButton();

    double xSpeed;
    double ySpeed;
    double angularVelocity;
    // double throttle;

    // SmartDashboard.putString("Drivetrain state", state.toString());


    // if(driveLineup){
    //   state = states.LINEUP;
    // }

    if(driveBalance){
     state = states.BALANCE;
    }

   

    // if(m_driverController.getXButton()){
    //  state = states.BALANCE;
    // }

    // if (m_driverController.getAButton()){
    //   SwerveDrive.isFieldCentric = false;
    // }

    if(driverInput){
      state = states.MANUAL;
    } 
    // else if (driverInput && m_driverController.getAButton()){
    //   state = states.CAMERA;
    // }

          
    // if (m_driverController.getXButton()){
    //   m_driverChoice = driverChoice.RESET;
    // }

    // SmartDashboard.putBoolean("linedUp", linedUp);
    // SmartDashboard.putBoolean("driverInput", driverInput);

    // if(m_driverController.getBButton()){
    //   state = states.LINEUP;
    // }

    switch(state){
//       case LINEUP:

//       //if (mainstates == frc.robot.commands.SimplerMainStateMachine.states.LINEUP_SCORE){
//       //  m_Limelight.chooseSide();
// //
//       //}

//       xSpeed = deadZone(m_Limelight.getY() * 1 * xMultiplier); // swapped X and Y so it corresponds with x and y of the field
//       ySpeed = deadZone(m_Limelight.getX() * -1 * yMultiplier);
//       double X_OUTPUT_CAP = 0.14;
//       double Y_OUTPUT_CAP = 0.07;

      
//       if(xSpeed > X_OUTPUT_CAP){
//         xSpeed = X_OUTPUT_CAP;
//       }
//       if(xSpeed < -X_OUTPUT_CAP){
//         xSpeed = -X_OUTPUT_CAP;
//       }
//       if(ySpeed > Y_OUTPUT_CAP){
//         ySpeed = Y_OUTPUT_CAP;
//       }
//       if(ySpeed < -Y_OUTPUT_CAP){
//         ySpeed = -Y_OUTPUT_CAP;
//       }
      
      
        
//         angularVelocity = findHeadingError(-m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees(), 0)/70.0;
//         SwerveDrive.isFieldCentric = true;

//         // SmartDashboard.putBoolean("Filtered Lineup", getFilteredLINEDUP());
//         if(getFilteredLINEDUP()){
//           state = states.MANUAL;
//         }

//       break;
        
      case MANUAL:
      SwerveDrive.isFieldCentric = true;

        if (m_Arm.armDrivetrainRamping() != 0){
          throttle = 1 / m_Arm.armDrivetrainRamping();
        }

        if(m_driverController.getBButton() || m_Arm.getPosition() > 1000){
          throttle = 0.35;
          // angularVelocity = 0.1;
          // SmartDashboard.putBoolean("slow button", m_driverController.getBButton()|| SimplerMainStateMachine.scoring && !(m_driverController.getRightTriggerAxis() < 0.3));
          // System

        } 
        else {
          throttle = 1 - ((m_driverController.getRightTriggerAxis()) * 0.3);
          // SmartDashboard.putBoolean("acceleration thing", m_driverController.getRightTriggerAxis() < 0.3);
          // System.out.println("true");
        }

        // if (m_driverController.getXButton()){
        //   SwerveDrive.isFieldCentric = false;
        //   throttle = 0.3;
        // } else {
        //   SwerveDrive.isFieldCentric = true;
        // }

        /*
         * if robot angle is smaller than deadzone,
         * save current angle and make it target angle 
         * keep the robot on target 
         * collection done only once when the robot is on the range 
         * when out of deadzone, changing the angle 
         */


        angularVelocity = (-DrivetrainConstants.ROBOTMAXSPEED * SwerveDrive.deadZone(m_driverController.getRightX()) * throttle) * 0.3;

        // if (SwerveDrive.deadZone(m_driverController.getRightX()) == 0){
        //   if (flag == true){
        //     targetAngle = m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees();
        //     flag = false;
        //   }
        // } else if (SwerveDrive.deadZone(m_driverController.getRightX()) > 0) {
        //   flag = true;
        // } else {
        //   angularVelocity = (findHeadingError(-m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees(), targetAngle)/69.96);
        // }

        if (m_driverController.getAButton()){// grid slow button 
          angularVelocity = (findHeadingError(-m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees(), 180)/65);
          throttle = 0.3;
        }

        if (m_driverController.getXButton()){ //single substation button
          angularVelocity = (findHeadingError(-m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees(), 90)/65);//69.96
          throttle = 0.3;
        }

        if (m_driverController.getRightBumper()){// lineup, parallel to double substation 
          m_Limelight.setPipeline(0);
          angularVelocity = (findHeadingError(-m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees(), 0)/65);//69.96
          // if (m_Arm.getPosition() < 450){
            
            if (m_Arm.getPosition() > 1000){
              throttle = 0.35;
            } else if (
              m_Limelight.getValid() && // too far away from apriltag 
              m_Limelight.getArea() <= 0.1 && 
              (m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees() > -10 && m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees() < 10)){
              throttle = 1;
            } else if (
              m_Limelight.getValid() && // entering double substation zone
              m_Limelight.getArea() >= 0.1 && 
              // m_Limelight.getArea() < 0.79 &&
              (m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees() > -10 && m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees() < 10)){
              throttle = 1 / (taMultiplier * m_Limelight.getArea());// drivetrain slows down as it approaches the apriltag (area gets bigger; inversely proportional)
            // } else if (
            //   m_Limelight.getValid() &&
            //   m_Limelight.getArea() >= 0.79 &&
            //   (m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees() > -10 && m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees() < 10)){
            //     throttle = 0.25;
            } else if (
              !m_Limelight.getValid() &&// not seeing apriltag at all 
              (m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees() > -10 && m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees() < 10)){
                throttle = 1; //0.35;
            }
            // } else if (m_Arm.getPosition() > 1000){
            //   throttle = 0.35;
            // }
            // } else {
            //   throttle = 0.35;
            // }
          }

          // if (m_driverController.getYButton()){
          //   angularVelocity = findHeadingError(-m_SwerveDrive.m_odometry.getPoseMeters().getRotation().getDegrees(), 0)/14;
          // }

        //should the trigger override
        // SmartDashboard.putBoolean("slow button", !(m_driverController.getBButton()|| SimplerMainStateMachine.scoring));
        // SmartDashboard.putBoolean("acceleration thing", m_driverController.getRightTriggerAxis() >= 0.3);

        // SmartDashboard.putNumber("throttle", throttle);

        xSpeed = -SwerveDrive.deadZone(m_driverController.getLeftY()) * throttle; // *0.3 // all
        ySpeed = -SwerveDrive.deadZone(m_driverController.getLeftX()) * throttle;        

        break;

      // case CAMERA:
      //   if(m_driverController.getBButton() || SimplerMainStateMachine.scoring){
      //     throttle = 0.35;
      //     // angularVelocity = 0.1;
      //   } 
      //   else{
      //     throttle = 1;
      //     // angularVelocity = 1;
      //   }

      //   xSpeed = -DrivetrainConstants.DEADZONEMULTIPLIER * SwerveDrive.deadZone(m_driverController.getLeftY()) * throttle; // *0.3 // all
      //   ySpeed = -DrivetrainConstants.DEADZONEMULTIPLIER * SwerveDrive.deadZone(m_driverController.getLeftX()) * throttle;
      //   angularVelocity = -DrivetrainConstants.DEADZONEMULTIPLIER * SwerveDrive.deadZone(m_driverController.getRightX());
      //   SwerveDrive.isFieldCentric = false;

      //   break;

      case BALANCE:
        double x_offset_angle = -SwerveDrive.imu.getRoll();
        double y_offset_angle = -SwerveDrive.imu.getPitch();

        // SmartDashboard.putNumber("x offset angle", -SwerveDrive.imu.getRoll());
        // SmartDashboard.putNumber("y offset angle", -SwerveDrive.imu.getPitch());

        SwerveDrive.isFieldCentric = false;

        xSpeed = x_offset_angle * 0.0065;
        ySpeed = y_offset_angle * 0.0065;
        angularVelocity = 0;

        if(x_offset_angle < 2 && x_offset_angle > -2 && y_offset_angle < 2 && y_offset_angle > -2){
          state = states.BALANCED;
        }

        break;  

      case BALANCED:
        
        m_SwerveDrive.backLeft.motorSteer.set(ControlMode.Position,135);
        m_SwerveDrive.frontLeft.motorSteer.set(ControlMode.Position,135);
        m_SwerveDrive.backRight.motorSteer.set(ControlMode.Position,45);
        m_SwerveDrive.frontLeft.motorSteer.set(ControlMode.Position,45);

        xSpeed = 0;
        ySpeed = 0;
        angularVelocity = 0;

        break;

        //case CAMERA:
  
      default:
        xSpeed = 0;
        ySpeed = 0;
        angularVelocity = 0;
        break;
        
    }
  
    m_SwerveDrive.drive(xSpeed, ySpeed, angularVelocity, SwerveDrive.isFieldCentric);
    SmartDashboard.putBoolean("Apriltag visible", m_Limelight.getValid());
    // SmartDashboard.putNumber("Arm ramping", m_Arm.armDrivetrainRamping());
    SmartDashboard.putNumber("Speed (throttle)", throttle);
    //TODO print target angle 
  }

  private Timer filterTimer = new Timer();

  public boolean getFilteredLINEDUP(){
    boolean filteredLINEDUP = false;
    if(linedUp && filterTimer.hasElapsed(0.009)){
      filteredLINEDUP = true;
    }
    else if(!linedUp){
      filterTimer.reset();
    }
    // SmartDashboard.putNumber("Filter timer", filterTimer.get());
    // SmartDashboard.putBoolean("advance method", filterTimer.advanceIfElapsed(0.0025));
    return filteredLINEDUP;
  }

  public void stateUpdate(states targetState){
    state = targetState;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double deadZone(double input){
    if(Math.abs(input) < 0.04){
      return 0.0;
    }
    return input;
  } 

  private double findHeadingError(double currentHeading, double targetHeading){
    currentHeading %= 360;
    targetHeading %= 360;
    if(currentHeading < 0){
      currentHeading += 360;
    }
    if(targetHeading < 0 ){
      targetHeading += 360;
    }

    double difference = currentHeading - targetHeading;
    if (difference > 180){
      return(-360) + difference;
    }
    else if (difference < -180){
      return 360 + difference;
    }
    else {
      return difference;
    }
  }
}
