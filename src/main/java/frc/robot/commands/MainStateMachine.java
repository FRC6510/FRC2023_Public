// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ArmReal;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Wrist;
// import frc.robot.subsystems.LiftReal;
// import frc.robot.subsystems.Drivetrain.SwerveDrive;
// import frc.robot.subsystems.OuttakeReal;
// import frc.robot.commands.SwerveDriveStateCommand;

// import com.revrobotics.CANSparkMax.IdleMode;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class MainStateMachine extends CommandBase {
//   private LiftReal m_Lift;
//   private ArmReal m_Arm;
//   private Limelight m_Limelight;
//   private Wrist m_Wrist;
//   private OuttakeReal m_Outtake;
//   private SwerveDriveStateCommand m_SwerveDriveStateCommand;
//   private XboxController m_operatorController;

//   boolean operatorInput;

//   boolean intakeCube;
//   boolean intakeCone;
//   boolean outtakeCube;
//   boolean outtakeCone;

//   boolean hybridPos;//the following 3 all depend on the previous 4 booleans.
//   boolean mediumPos;
//   boolean highPos;

//   boolean stored;

//   public enum gamepieces{
//     CONE,
//     CUBE;
//   }

//   public static gamepieces m_gamepiece = gamepieces.CONE;

//   public enum states{
//     IDLE, //default, does nothing
//     DETECT_PICKUP, //detect apriltag for substation
//     LINEUP_PICKUP, //line up to apriltag, move arm and lift to intake position
//     INTAKE, // move intake motors 
//     LINEUP_DELIVER, // line up with apriltag/reflective tape ->  use conditionals with stored data abt what gamepieces are already there to choose cube or cone column
//     // DETECT_DELIVER, // detecting apriltag/reflective tape -> used stored data plus vision??? to know which level to deliver gampepiece to; move all subsystems to target position 
//     OUTTAKE; // move outake motor
//   //needs an option for picking up from  a non-substation
//   }

//   static states m_state = states.IDLE;

//   enum pickupPositions{
//     NONE,
//     SINGLE,
//     DOUBLE,
//     FLOOR;
//   }
//   static pickupPositions m_pickupPosition = pickupPositions.NONE;

//   /** Creates a new MainStateMachine. */
//   public MainStateMachine(LiftReal m_Lift, ArmReal m_Arm, Limelight m_Limelight, Wrist m_Wrist, OuttakeReal m_Outtake, SwerveDriveStateCommand m_SwerveDriveStateCommand, XboxController m_driverController, XboxController m_operatorController) {
//     this.m_Lift = m_Lift;
//     this.m_Arm = m_Arm;
//     this.m_Limelight = m_Limelight;
//     this.m_Wrist = m_Wrist;
//     this.m_Outtake = m_Outtake;
//     this.m_SwerveDriveStateCommand = m_SwerveDriveStateCommand;
//     // this.m_driverController = m_driverController;
//     this.m_operatorController = m_operatorController;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(this.m_Arm, this.m_Limelight, this.m_Wrist, this.m_Outtake);
//     // We don't need to add the SwerveDriveStateCommand nor the controllers as requirements. 
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     stored = false;
//     // intakeCube = m_operatorController.getRightBumper();
//     // intakeCone = m_operatorController.getLeftBumper();

//     if (m_operatorController.getAButton()){

//       m_pickupPosition = pickupPositions.FLOOR;
//       m_gamepiece = gamepieces.CONE;

//     } else if (m_operatorController.getBButton()){

//       m_pickupPosition = pickupPositions.SINGLE;
//       m_gamepiece = gamepieces.CONE;

//     } else if (m_operatorController.getYButton()){

//       m_pickupPosition = pickupPositions.DOUBLE;
//       m_gamepiece = gamepieces.CONE;

//     } else if (m_operatorController.getXButton()){

//       m_pickupPosition = pickupPositions.FLOOR;
//       m_gamepiece = gamepieces.CUBE;

//     }

//     switch(m_state){
//       case IDLE:
      
//       m_pickupPosition = pickupPositions.NONE;
      
//       // TODO this is where you bring everything home (pls bring everything home)
//       m_Lift.setLiftMin();
//       m_Arm.setArm(0);
//       m_Wrist.setWrist(0);

//       // if (intakeCube){
//       //   m_state = states.INTAKE;
//       // } else if (intakeCone) {
//       //   m_state = states.INTAKE;
//       // }

//       if (m_pickupPosition != pickupPositions.NONE){// if a collect position button has been pressed
//         m_state = states.LINEUP_PICKUP;
//       }

//         // set all target positions to default (starting/default running positions
//       if(stored){
//         //something stored, start outtake progression
//         //wait for driver to select to progress to lining up or 
//         if (m_SwerveDriveStateCommand.driveLineup){//there is only 1 button for automatic scoring -> from here, the loop takes over 
//           m_state = states.LINEUP_DELIVER;
//         }
//       }
//       else{// nothing stored, start intake progression
//         //picking a substation, picking the target object --> will be an operator input
//         if (m_SwerveDriveStateCommand.driveLineup){//the button should be held down by the driver - seek apriltag continuously 
//           m_state = states.DETECT_PICKUP;
//         }
//       }
//         break;    
//       case DETECT_PICKUP:
//         //get value from limelight subsystem 
//         /*
//          * 1. set pipeline for the apriltag in the substation
//          * 2. check to see if any apriltags are detected
//          * 3. if apriltag detected, go to LINEUP_PICKUP. If not, go back to IDLE
//          */

//        m_Limelight.setPipeline(m_Limelight.doubleSubstationPipeline);//TODO Change this to limelight 'autoselect' which Mulan will code 
//        m_SwerveDriveStateCommand.stateUpdate(frc.robot.commands.SwerveDriveStateCommand.states.LINEUP);
//        if (m_Limelight.getValid() || m_pickupPosition != pickupPositions.NONE){// if aligned OR if a lineup position button has been pressed 
//         m_state = states.LINEUP_PICKUP;
//        } else {
//         m_state = states.IDLE;
//        }

//        //  Limelight.setPipeline(0);//TODO change accordingly      
//        //  if (Limelight.getValid()){
//        //    state = states.LINEUP_PICKUP;
//        //  } else {
//        //    state = states.IDLE;
//        //  }
//         break;    
//       case LINEUP_PICKUP:      
//         //move all subsystems into pick-up position
//         //pick-up position will be dependent on:
//         /*
//          * 1. if we want a cone or a cube 
//          * 2. if we want the shelf or the floor (different substations) --> accessed with the pickupPositions enum 
//          */
//         //line up robot drivetrain relative to the loc. of the apriltag 
//         //progressed when: drivetrain is lined up AND all subsystems in position       
       
       
//        switch (m_gamepiece){

//         case CUBE:
//           m_Arm.setArm(0);

//         break;

//         case CONE:

//           switch (m_pickupPosition){

//             case SINGLE:

//               m_Arm.setArm(0);
//               m_Wrist.setWrist(0);

//             break;

//             case DOUBLE:
//               m_Arm.setArm(90);
//               m_Wrist.setWrist(90);

//             break;

//             case FLOOR://TODO measurements from Jas 

//               m_Wrist.setWrist(-50);//check if this is right 
//               m_Arm.setArm(11);//check if this is right 

//             break;

//             case NONE:

//             break;

//           }

//         break;
//        }

//         if (m_SwerveDriveStateCommand.getLINEDUP()){
//           m_state = states.INTAKE;
//         }

//        //  if (SwerveDriveStateCommand.getLINEDUP()){
//        //    state = states.INTAKE;
//        //  }
//         break; 
//       case INTAKE:
//         //move the intake motors; the arm and wrist to lower the intake on to the object; attempt to intake object
//         //progress once we know we have completed the intake --> how much voltage the motors are draining OR a distance sensor       
        
//         if (m_gamepiece == gamepieces.CUBE){// TODO fill this in with the correct logic 
//           m_Outtake.setSpeed(0.7);// TODO needs to be changed so it runs on position, not speed

//           if (m_Outtake.objectDetected){
//             stored = true;
//             m_state = states.IDLE;
//           }
//         } else if (m_gamepiece == gamepieces.CONE){
//           m_Outtake.setSpeed(-0.7);
//           if (m_Outtake.objectDetected){
//             stored = true;
//             m_state = states.IDLE;
//           }
//         }

//       //  m_Outtake.setSpeed(0.7); 
//       //  if (m_Outtake.objectDetected){
//       //    stored = true;
//       //    m_state = states.IDLE;
//       //  }

//        //  OuttakeReal.setSpeed(0.7);
//        //  if (OuttakeReal.objectDetected){
//        //    stored = true;
//        //    state = states.IDLE;
//        //  }
//         break;    
//       case LINEUP_DELIVER:
//         //move all subsystems to a ready-to-deliver position 
//         //align the robot drivetrain with the apriltag 
//         //select an apriltag pipeline based off of what we have stored (cone, cube)
//         //check to see if there are any viable targets 
//         //if there is, change state to DETECT_DELIVER; if not, change state to IDLE      
        
        //switch(m_pickupPosition){
//
        //  case NONE: 
        //  //
        //  break;
//
        //  case SHELF:
        //  if (linedUp){
        //    m_SwerveDriveStateCommand.stateUpdate(frc.robot.commands.SwerveDriveStateCommand.states.LINEUP);
        //  }
        //  m_Arm.setArm(10);
        //  m_Wrist.setWrist(30);
//
//
        //    break;
        //  
        //  case FLOOR:
        //  m_SwerveDriveStateCommand.stateUpdate(frc.robot.commands.SwerveDriveStateCommand.states.LINEUP);
        //  m_Arm.setArm(0);
        //  m_Wrist.setWrist(0);
//
        //    break;
//
        //}
        //
//        //  Limelight.setPipeline(1);//TODO change accordingly
//        //  SwerveDriveStateCommand.stateUpdate(frc.robot.commands.SwerveDriveStateCommand.states.LINEUP);//access the SwerveDriveStateCommand, line up to apriltag      
//        //  if (Limelight.getValid()){
//        //    state = states.DETECT_DELIVER;
//        //  } else {
//        //    state = states.IDLE;
//        //  }
//         break;    
//       // case DETECT_DELIVER:
//       //   //try to detect the highest available height 
//       //   //if there is none (for cone, no retroflective tape; for cube, use vision to see if there are any cubes) --> need to make your own pipeline/vision control
//       //   //if no available target, move robot to another position
//       //   //have an array for all the delivery positions the robot has moved to (left, mid, right)
//       //   //if available position, select highest one 
//       //   //then move all subsystems to target position 
//       //   //TODO 
//       //   m_state = states.OUTTAKE;
//       //   break;
//       case OUTTAKE:
//         //spin intake rollers backwards; set a speed 
//         //stop when: voltage is normal OR use a timer OR use a proximity sensor
        
//         outtakeCube = m_operatorController.getLeftBumper();
//         outtakeCone = m_operatorController.getRightBumper();
//         // write logic to reject the 'wrong' button (reference teams)

//         m_Outtake.setSpeed(0.7);
//         if (!m_Outtake.objectDetected){
//          stored = false;
//          m_state = states.IDLE;
//         }
        
//        //  OuttakeReal.setSpeed(0.7);    
//        //  if (!OuttakeReal.objectDetected){
//        //    stored = false;
//        //    state = states.IDLE;
//        //  } 
//         break;
//     }

//     SmartDashboard.putString("Main state", m_state.toString());
//     SmartDashboard.putString("Gampiece", m_gamepiece.toString());
//     SmartDashboard.putString("Pickup position", m_pickupPosition.toString());
  
  //}
  

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
