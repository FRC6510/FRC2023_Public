// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.ArmReal;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Lift.LiftReal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
//import frc.robot.subsystems.LEDs;

import frc.robot.subsystems.Drivetrain.SwerveDrive;
import frc.robot.commands.Drivetrain.ProfiledMoveTo;
import frc.robot.commands.SwerveDriveStateCommand.driverChoice;
import frc.robot.subsystems.OuttakeReal;
import edu.wpi.first.wpilibj.XboxController;

public class SimplerMainStateMachine { //extends CommandBase {
  private LiftReal m_Lift;
  private ArmReal m_Arm;
  private Limelight m_Limelight;
  private Wrist m_Wrist;
  private OuttakeReal m_Outtake;
  private SwerveDriveStateCommand m_SwerveDriveStateCommand;
  private XboxController m_operatorController;
  private XboxController m_driverController;
  private Timer m_Timer;
  private SwerveDrive m_SwerveDrive;
  //private LEDs m_LEDS;

  public static boolean stored;
  public static boolean scoring;

  public enum gamepieces{
    NONE,
    CONE,
    CUBE;
  }
  public static gamepieces m_gamepiece = gamepieces.NONE;

  // public enum driverChoice{
  //   DRIVER_NONE,
  //   DRIVER_RESETUP,
  //   DRIVER_RESETDOWN;
  // }
  // public static driverChoice m_driverChoice = driverChoice.DRIVER_NONE;

  public enum scoringPosition{
    NONE,
    LOW_CONE,
    MID_CONE,
    HIGH_CONE,
    LOW_CUBE,
    MID_CUBE,
    HIGH_CUBE;
  }

  public static scoringPosition m_scoringPosition = scoringPosition.NONE;

  public enum states{
    IDLE, //default, does nothing
    LINEUP_COLLECT,
    MOVE_TO_COLLECT,
    LINEUP_SCORE;
  }

  static states m_state = states.IDLE;

  enum pickupPositions{
    NONE,
    SINGLE,
    DOUBLE,
    FLOOR;
  }
  static pickupPositions m_pickupPosition = pickupPositions.NONE;

  /** Creates a new SimplerMainStateMachine. */
  public SimplerMainStateMachine(
    LiftReal m_Lift, 
    ArmReal m_Arm, 
    Limelight m_Limelight, 
    Wrist m_Wrist, 
    OuttakeReal m_Outtake, 
    SwerveDriveStateCommand m_SwerveDriveStateCommand, 
    XboxController m_driverController, 
    XboxController m_operatorController, 
    SwerveDrive m_SwerveDrive
    //LEDs m_LEDS
  ){
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Lift = m_Lift;
    this.m_Arm = m_Arm;
    this.m_Limelight = m_Limelight;
    this.m_Wrist = m_Wrist;
    this.m_Outtake = m_Outtake;
    this.m_SwerveDriveStateCommand = m_SwerveDriveStateCommand;
    this.m_driverController = m_driverController;
    this.m_operatorController = m_operatorController;
    this.m_SwerveDrive = m_SwerveDrive;
    //this.m_LEDS = m_LEDS;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(this.m_Arm, this.m_Limelight, this.m_Wrist, this.m_Outtake);
    // We don't need to add the SwerveDriveStateCommand nor the controllers as requirements. 
  }

  // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  public void execute() {
    m_Timer = new Timer();
    stored = !m_Outtake.outtaken;
    scoring = false;
    // collection positions 

    // if (m_driverController.getPOV() == 0){
    //   m_driverChoice = driverChoice.DRIVER_RESETUP;
    // }

    // if (m_driverChoice == driverChoice.DRIVER_RESETUP){
    //   m_Lift.setLift(44000, 0.25);
    //   if (m_Lift.getPosition() > 43700){
    //     m_driverChoice = driverChoice.DRIVER_RESETDOWN;
    //   }
    // }
    // if (m_driverChoice == driverChoice.DRIVER_RESETDOWN){
    //   m_Lift.setLift(10, 0.25);
    //   m_driverChoice = driverChoice.DRIVER_NONE;
    // }

    if (m_operatorController.getAButton()){

      m_pickupPosition = pickupPositions.FLOOR;
      m_gamepiece = gamepieces.CONE;// pickup position for floor cone 

      double armFloor = 11865;// this is for the above 2 lines 
      double wristFloor = 36000; //37742;
      double liftFloor = m_Lift.liftMin;

      // m_Arm.setArmMotionMagic(9535);//check if this is right 
      m_Lift.setLift(liftFloor, 0.25);
      m_Arm.setArm(armFloor, 0.6);
      m_Wrist.setWrist(wristFloor, 0.6);//check if this is right 

      // A: 9535
      // W: -36909

    } else if (m_operatorController.getBButton()){

      m_pickupPosition = pickupPositions.SINGLE;
      m_gamepiece = gamepieces.CONE;
      double armSingle = 487;
      double wristSingle = 3822;
      double liftFloor = m_Lift.liftMin; 

      m_Lift.setLift(liftFloor, 0.25);
      m_Arm.setArm(armSingle, 0.6);
      m_Wrist.setWrist(wristSingle, 0.6); 

    } else if (m_operatorController.getYButton()){

      m_pickupPosition = pickupPositions.DOUBLE;
      m_gamepiece = gamepieces.CONE;
      double armDouble = 93332;
      double wristDouble = 68776.0;
      double liftFloor = m_Lift.liftMin; 

      // m_Arm.setArmMotionMagic(93332.0);
      m_Lift.setLift(liftFloor, 0.25);
      m_Arm.setArm(armDouble - 4500, 0.75);
      m_Wrist.setWrist(wristDouble, 0.6);

      // A: 93332.0

      // W: -68776.0

    } else if (m_operatorController.getXButton()){

      m_pickupPosition = pickupPositions.FLOOR;
      m_gamepiece = gamepieces.CUBE;

      double armFloor = 487;
      double wristFloor = 18500; //19000; //20000; //18190;//24761.0;
      double liftFloor = m_Lift.liftMin; 

      // m_Arm.setArmMotionMagic(487);
      m_Lift.setLift(liftFloor, 0.25);
      m_Arm.setArm(armFloor, 0.6);
      m_Wrist.setWrist(wristFloor, 0.6);//check if right 

      // A: 487
      // W: 24761.0

    }

    m_Outtake.spinIntake(1, m_operatorController);
 
    // m_LEDS.changeColour(m_operatorController);//changing LED colour manually with operator controller

    // if (m_Outtake.hasPurple){
    //   m_LEDS.turnPurple();
    //   // SmartDashboard.putString("led purple", "has purple");
    // } else if (m_Outtake.hasYellow){
    //   m_LEDS.turnYellow();
    //   // SmartDashboard.putString("led yellow", "has yellow");
    // } else if (m_LEDS.wantpurple){
    //   m_LEDS.purpleFast();

    // } else if (m_LEDS.wantyellow){
    //   m_LEDS.yellowFast();

    // } else {
    //   m_LEDS.resetAnimation();
    //   SmartDashboard.putString("led resetting", "is resetting");
    // }
    // // m_Outtake.intakeAdjustment(0.3, m_driverController);
    score();

    //idle position logic 
    if ((!m_Outtake.outtaken && (m_scoringPosition == scoringPosition.NONE)) ||// to idle after intaking 
        (m_Outtake.outtaken) && (m_scoringPosition != scoringPosition.NONE)){// to idle after scoring 
          if (m_Outtake.returnTimer() > 0.3){// delay
            m_Lift.setLiftMin();
            m_Wrist.setWrist(0, 0.6);
            m_Arm.setArm(0, 0.7);
            m_scoringPosition = scoringPosition.NONE;
          }
    } else if (m_gamepiece == gamepieces.NONE){
      m_Lift.setLiftMin();
      m_Wrist.setWrist(0, 0.6);
      m_Arm.setArm(0, 0.7);
    }

    // switch (m_state){
    //   case IDLE:        
    //     if(stored){
    //       //something stored, start outtake progression
    //       //wait for driver to select to progress to lining up or 
    //       if (m_SwerveDriveStateCommand.driveLineup){//there is only 1 button for automatic scoring -> from here, the loop takes over 
    //         m_state = states.LINEUP_SCORE;
    //       }
    //     }
    //     else{// nothing stored, start intake progression
    //       //picking a substation, picking the target object --> will be an operator input
    //       if (m_SwerveDriveStateCommand.driveLineup){//the button should be held down by the driver - seek apriltag continuously 
    //         m_state = states.LINEUP_COLLECT;
    //       }
    //     }

    //     m_SwerveDriveStateCommand.stateUpdate(frc.robot.commands.SwerveDriveStateCommand.states.MANUAL);

    //     break;  

    //     case LINEUP_COLLECT:
    //       // double armDouble = 93332;
    //       // double wristDouble = 68776.0;

    //       // m_Arm.setArm(armDouble - 5000);
    //       // m_Wrist.setWrist(wristDouble, 0.6);

    //       m_Limelight.setPipeline(m_Limelight.doubleSubstationPipeline);//TODO Change this to limelight 'autoselect' which Mulan will code 
    //       m_SwerveDriveStateCommand.stateUpdate(frc.robot.commands.SwerveDriveStateCommand.states.LINEUP);
          
    //       if (!m_SwerveDriveStateCommand.driveLineup){
    //         // m_Lift.setLiftMin();
    //         // m_Wrist.setWrist(0, 0.6);
    //         // m_Arm.setArm(0);
    //         //m_gamepiece = gamepieces.NONE; // what's this for??
    //         m_state = states.IDLE;
    //       }

    //       if(m_SwerveDriveStateCommand.getFilteredLINEDUP()){
    //         m_state = states.MOVE_TO_COLLECT;
    //         double X_OFFSET = 0.0;
    //         double Y_OFFSET = -1;
    //         ProfiledMoveTo alignWithDoubleStation = new ProfiledMoveTo(m_SwerveDrive, m_SwerveDrive.m_odometry.getPoseMeters().plus(new Transform2d(new Translation2d(X_OFFSET, Y_OFFSET), new Rotation2d(0))), 2);
    //         alignWithDoubleStation.schedule();
    //       }


    //     break;    

    //     case LINEUP_SCORE:
          
    //       m_Limelight.setPipeline(0);
    //       m_SwerveDriveStateCommand.stateUpdate(frc.robot.commands.SwerveDriveStateCommand.states.LINEUP);
    //       if (!m_SwerveDriveStateCommand.driveLineup){
    //         m_state = states.IDLE;
    //       }

    //     break;

    //     case MOVE_TO_COLLECT:
    //       // relative profile move to
          
    //       //driver presses cone intake button themselves
    //       // m_gamepiece = gamepieces.CONE;
      
    //     break;
    // }

    // SmartDashboard.putString("1Main state", m_state.toString());
    // SmartDashboard.putString("1Gamepiece", m_gamepiece.toString());
    // SmartDashboard.putString("1Pickup position", m_pickupPosition.toString());
    // SmartDashboard.putString("1Scoring position", m_scoringPosition.toString());

    // SmartDashboard.putBoolean("Want P", m_LEDS.wantpurple);
    // SmartDashboard.putBoolean("Want Y", m_LEDS.wantyellow);
    // SmartDashboard.putBoolean("Has P", m_Outtake.hasPurple);
    SmartDashboard.putBoolean("Has Y", m_Outtake.hasYellow);

  }

  public void score(){
    if (m_operatorController.getPOV() == 0){// high 
      switch (m_gamepiece){
        case CUBE:// High cube

        // TODO limit the drivetrain speed 

        m_scoringPosition = scoringPosition.HIGH_CUBE;

        double armHighCube = 73354.0;
        double liftHighCube = 39008;
        double wristHighCube = 32316;

        // m_Arm.setArmMotionMagic(73354.0);
        m_Arm.setArm(armHighCube, 0.6);
        m_Lift.setLift(liftHighCube, 0.25);
        m_Wrist.setWrist(wristHighCube, 0.55);
      
        // A: 73354
        // W: -32316
        // L: 39008

        break;

        case CONE:// High cone
        m_scoringPosition = scoringPosition.HIGH_CONE;
        
        double liftHighCone = 42799;//42399;// new tuning on Wed
        double armHighCone = 106461;//108461;// new tuning on Wed
        double wristHighCone = 83900 + 2050;//2950;// new tuning on Wed

        // m_Arm.setArmMotionMagic(109884.0);
        // m_Timer.reset();
        // m_Timer.start();
        m_Lift.setLift(liftHighCone, 0.25);
        m_Arm.setArm(armHighCone, 0.6);//99884
        // if (m_Arm.getPosition() >= (armHighCone / 2)){
        //   m_Wrist.setWrist(wristHighCone);
        // }
        m_Wrist.setWrist(wristHighCone, 0.6);//2000
        // m_Timer.get();
        // if (m_Timer.get() > 0.001){
        //   m_Wrist.setWrist(wristHighCone);
        // }
        //A: 109884.0
        //W: -73912.

        break;

        case NONE:

        m_Arm.setArm(0, 0.6);
        m_Wrist.setWrist(0, 0.25);
        m_Lift.setLift(0,0.24); 

        break;

        default:

        m_Arm.setArm(0, 0.6);
        m_Wrist.setWrist(0, 0.25);
        m_Lift.setLift(0,0.24); 
        
        break;
      }
    } else if (m_operatorController.getPOV() == 90 || m_operatorController.getPOV() == 270){ // mid
      switch (m_gamepiece){
        case CUBE:// Mid cube
        m_scoringPosition = scoringPosition.MID_CUBE;
        double liftMidCube = 39008; 
        double armMidcube = 46004.0;
        double wristMidCube = 20319; 

        // TODO limit the drivetrain speed 

        // m_Arm.setArmMotionMagic(46004.0);
        m_Arm.setArm(armMidcube, 0.6);
        m_Lift.setLift(liftMidCube, 0.25);
        m_Wrist.setWrist(wristMidCube, 0.6);

        // A: 46004
        // W: -20319
        // L: 39008

        break;

        case CONE:// Mid cone
        m_scoringPosition = scoringPosition.MID_CONE;
        double liftMidCone = 24; //can delete if needed
        double armMidCone = 96643.0;
        double wristMidCone = 85482;

        // m_Arm.setArmMotionMagic(106643.0);
        // m_Arm.setArm(106643.0);
        // m_Timer.reset();
        // m_Timer.start();
        m_Arm.setArm(armMidCone, 0.6);
        // m_Timer.get();
        //if (m_Timer.get() > 0.001){
        m_Wrist.setWrist(wristMidCone, 0.25);
        m_Lift.setLift(liftMidCone,0.24); //testing
        //}
        // if (m_Arm.getArm() > (armMidCone / 2)){
        //   m_Wrist.setWrist(wristMidCone);
        // }

        //A: 106643.0
        //W: -85482.0

        break;
        
        case NONE:

        m_Arm.setArm(0, 0.6);
        m_Wrist.setWrist(0, 0.25);
        m_Lift.setLift(0,0.24);

        break;

        default:

        m_Arm.setArm(0, 0.6);
        m_Wrist.setWrist(0, 0.25);
        m_Lift.setLift(0,0.24);

        break;
      }

    } else if (m_operatorController.getPOV() == 180){ // low
      switch (m_gamepiece){
        case CUBE:// low cube
        m_scoringPosition = scoringPosition.LOW_CUBE;
        double armLowCube = 487.0;
        double wristLowCube = 0;

        // m_Arm.setArmMotionMagic(487.0);
        m_Arm.setArm(armLowCube, 0.6);
        m_Wrist.setWrist(wristLowCube, 0.6);

        //A: 487
        //W: 0

        break;

        case CONE:// low cone 
        m_scoringPosition = scoringPosition.LOW_CONE;
        double armLowCone = 487.0;
        double wristLowCone = 0;

        // m_Arm.setArmMotionMagic(487.0);
        m_Arm.setArm(armLowCone, 0.6);
        m_Wrist.setWrist(wristLowCone, 0.6);

        //A: 487
        //W: 0

        break;

        case NONE:

        m_Arm.setArm(0, 0.6);
        m_Wrist.setWrist(0, 0.25);
        m_Lift.setLift(0,0.24); 

        break;

        default:
        m_Arm.setArm(0, 0.6);
        m_Wrist.setWrist(0, 0.25);
        m_Lift.setLift(0,0.24); 
        break;
      }

    } 

    SmartDashboard.putNumber("Arm position", m_Arm.getPosition());
    //SmartDashboard.putData("gamepiece", gamepieces.);
  }

  public void resetStates(){
      m_gamepiece = gamepieces.NONE;
      m_pickupPosition = pickupPositions.NONE;
      m_state = states.IDLE;
  } 

  // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
