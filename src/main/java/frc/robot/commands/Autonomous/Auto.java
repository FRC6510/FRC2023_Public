package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.ProfiledMoveTo;
import frc.robot.commands.Drivetrain.ProfiledRotation;
import frc.robot.commands.LiftGo;
import frc.robot.commands.Autonomous.SetLiftAuto;
import frc.robot.commands.Autonomous.ArmWristAutoFloorCone;
import frc.robot.commands.Autonomous.AutoHome;
// import frc.robot.subsystems.Lift.LiftReal;
// import frc.robot.subsystems.ArmReal;
// import frc.robot.subsystems.Wrist;
// import frc.robot.subsystems.OuttakeReal;
public class Auto {
    RobotContainer m_robotContainer;
    // LiftReal m_Lift;
    // ArmReal m_Arm;
    // Wrist m_Wrist;
    // OuttakeReal m_Outtake;

    public Auto(RobotContainer m_robotContainer){
        this.m_robotContainer = m_robotContainer;
    }

    public static enum AutoList {
        AUTO_TEST,
        AUTO_BLUE_LEFT,
        AUTO_BLUE_LEFT_NO_TURN,
        AUTO_BLUE_RIGHT,
        AUTO_RED_LEFT,
        AUTO_RED_RIGHT,
        AUTO_RED_RIGHT_NO_TURN,
        AUTO_JUST_BALANCE,
        AUTO_BALANCE_NO_MOBILITY,
        AUTO_JUST_CUBE,
        AUTO_CONE_RED_RIGHT_CRAZYARM,
        AUTO_CONE_BLUE_LEFT_CRAZYARM,
        AUTO_DRIVE_TEST,
        AUTO_SCORE_TEST;
    }

    public CommandBase getAuto(AutoList auto){

        // m_Lift = new LiftReal();
        // m_Arm = new ArmReal();
        // m_Wrist = new Wrist();

        // m_Outtake = new OuttakeReal();

        switch(auto) {
            case AUTO_TEST:
            return Commands.sequence(
                new PrintCommand("anything"),
                new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(0, 0, //edge/corner of line
                    new Rotation2d(0))), m_robotContainer.swerveDrive),
                //new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(1,0, Rotation2d.fromDegrees(0)),1),
                new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(0,0, Rotation2d.fromDegrees(180)),1)
            );
            case AUTO_BLUE_LEFT:
            return Commands.sequence(                                                         //2.81
                new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(AutoConstants.starting_x, AutoConstants.community_leftCone_y - 0.1, //-0.55
                    new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position               
                    new SetLiftAuto(m_robotContainer.m_Lift),// moves )arm out from starting position 
                    new ArmWristAutoFloorCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Outtake).alongWith(
                        new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_leftCone_y,Rotation2d.fromDegrees(0)),1.251).raceWith(
                        new IntakeMoveStop(m_robotContainer.m_Outtake).withTimeout(10))// moves to left cone
                    ),// arm and wrist to floor cone intake position 
                    new WaitCommand(0.05),// get cone 
                    new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist),
                    new ProfiledRotation(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_leftCone_y,Rotation2d.fromDegrees(180)),1.261)//turn to face grid
                    ,// arm and wrist comes home 
                    // new WaitCommand(0.1),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(3.5,AutoConstants.scoring1_y - 0.2,Rotation2d.fromDegrees(180)),1).withTimeout(3.5), //move to community
                    new WaitCommand(0.03),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.scoring_x + 0.1,AutoConstants.scoring1_y,Rotation2d.fromDegrees(180)),0.5).alongWith(
                        new ArmWristScoreMidCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Lift).withTimeout(3.5)
                    ), //move to right node
                    new WaitCommand(0.1),
                    new OuttakeMoveStop(m_robotContainer.m_Outtake),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d((AutoConstants.scoring_x + 0.3),AutoConstants.scoring1_y,Rotation2d.fromDegrees(180)),0.5),
                    new WaitCommand(0.1),
                    new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist)
                    );

            case AUTO_BLUE_LEFT_NO_TURN:
            return Commands.sequence(                                                         //2.81
                new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(AutoConstants.starting_x, AutoConstants.community_leftCone_y - 0.1, //-0.55
                    new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position               
                    new SetLiftAuto(m_robotContainer.m_Lift),// moves )arm out from starting position 
                    new ArmWristAutoFloorCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Outtake).alongWith(
                        new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_leftCone_y,Rotation2d.fromDegrees(0)),1.251).raceWith(
                        new IntakeMoveStop(m_robotContainer.m_Outtake).withTimeout(10))// moves to left cone
                    ),// arm and wrist to floor cone intake position 
                    new WaitCommand(0.05),// get cone 
                    new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist),
                    new ProfiledRotation(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_leftCone_y,Rotation2d.fromDegrees(0)),1.261),//turn to face grid
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(3.5,AutoConstants.scoring1_y - 0.2,Rotation2d.fromDegrees(0)),1).withTimeout(3.5)
            );
    
            case AUTO_BLUE_RIGHT:
                return Commands.sequence(
                    new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(AutoConstants.starting_x, AutoConstants.community_rightCone_y, //-0.55
                    new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position               
                    new SetLiftAuto(m_robotContainer.m_Lift),// moves )arm out from starting position 
                    new ArmWristAutoFloorCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Outtake),// arm and wrist to floor cone intake position 
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_rightCone_y,Rotation2d.fromDegrees(0)),1.25).raceWith(
                    new IntakeMoveStop(m_robotContainer.m_Outtake).withTimeout(10)),// moves to left cone
                    new WaitCommand(0.1),// get cone 
                    new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist),// arm and wrist comes home 
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_rightCone_y,Rotation2d.fromDegrees(180)),1.25),//turn to face grid
                    // new WaitCommand(0.1),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(3.5,AutoConstants.scoring2_y,Rotation2d.fromDegrees(180)),1), //move to community
                    new WaitCommand(0.5),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.scoring_x + 0.05,AutoConstants.scoring2_y,Rotation2d.fromDegrees(180)),0.5).alongWith(
                        new ArmWristScoreMidCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Lift).withTimeout(3.5)
                    ), //move to right node
                    new WaitCommand(0.5),
                    new OuttakeMoveStop(m_robotContainer.m_Outtake),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.scoring_x + 0.35,AutoConstants.scoring2_y,Rotation2d.fromDegrees(180)),0.5),
                    new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist)           
                );

            case AUTO_RED_RIGHT:
                return Commands.sequence(
                    new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(AutoConstants.starting_x, AutoConstants.community_leftCone_y - 0.1, //-0.55
                    new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position               
                    new SetLiftAuto(m_robotContainer.m_Lift),// moves )arm out from starting position 
                    new ArmWristAutoFloorCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Outtake).alongWith(
                        new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_leftCone_y - 0.15,Rotation2d.fromDegrees(0)),1.251).raceWith(
                        new IntakeMoveStop(m_robotContainer.m_Outtake).withTimeout(10))// moves to left cone
                    ),// arm and wrist to floor cone intake position 
                    new WaitCommand(0.05),// get cone 
                    new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist).alongWith(
                        new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_leftCone_y - 0.15,Rotation2d.fromDegrees(180)),1.261)//turn to face grid
                    ),// arm and wrist comes home 
                    // new WaitCommand(0.1),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(3.5,AutoConstants.scoring1_y -0.55,Rotation2d.fromDegrees(180)),1).withTimeout(3.5), //move to community
                    new WaitCommand(0.03),

                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.scoring_x + 0.1,AutoConstants.scoring1_y - 0.65,Rotation2d.fromDegrees(180)),0.5).alongWith(
                        new ArmWristScoreMidCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Lift).withTimeout(3.5)
                    ), //move to right node
                    new WaitCommand(0.1),
                    new OuttakeMoveStop(m_robotContainer.m_Outtake),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d((AutoConstants.scoring_x + 0.3),AutoConstants.scoring1_y,Rotation2d.fromDegrees(180)),0.5),
                    new WaitCommand(0.1),
                    new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist)
                );

                case AUTO_RED_RIGHT_NO_TURN:
                return Commands.sequence(
                    new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(AutoConstants.starting_x, AutoConstants.community_leftCone_y - 0.1, //-0.55
                    new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position               
                    new SetLiftAuto(m_robotContainer.m_Lift),// moves )arm out from starting position 
                    new ArmWristAutoFloorCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Outtake).alongWith(
                        new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_leftCone_y - 0.2,Rotation2d.fromDegrees(0)),1.251).raceWith(
                        new IntakeMoveStop(m_robotContainer.m_Outtake).withTimeout(10))// moves to left cone
                    ),// arm and wrist to floor cone intake position 
                    new WaitCommand(0.05),// get cone 
                    new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist),
                    // new WaitCommand(0.1),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(3.5,AutoConstants.scoring1_y -0.55,Rotation2d.fromDegrees(0)),1)
                    
                );

                   
            // case AUTO_RED_LEFT:
            // return Commands.sequence(
            //     new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(AutoConstants.starting_x, AutoConstants.community_rightCone_y, //-0.55
            //     new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position               
            //     new SetLiftAuto(m_robotContainer.m_Lift),// moves )arm out from starting position 
            //     new ArmWristAutoFloorCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Outtake),// arm and wrist to floor cone intake position 
            //     new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_rightCone_y,Rotation2d.fromDegrees(0)),1.25).raceWith(
            //     new IntakeMoveStop(m_robotContainer.m_Outtake).withTimeout(10)),// moves to left cone
            //     new WaitCommand(0.1),// get cone 
            //     new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist),// arm and wrist comes home 
            //     new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_rightCone_y,Rotation2d.fromDegrees(180)),1.25),//turn to face grid
            //     // new WaitCommand(0.1),
            //     new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(3.5,AutoConstants.scoring2_y,Rotation2d.fromDegrees(180)),1), //move to community
            //     new WaitCommand(0.5),
            //     new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.scoring_x + 0.05,AutoConstants.scoring2_y,Rotation2d.fromDegrees(180)),0.5).alongWith(
            //         new ArmWristScoreMidCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Lift).withTimeout(3.5)
            //     ), //move to right node
            //     new WaitCommand(0.5),
            //     new OuttakeMoveStop(m_robotContainer.m_Outtake),
            //     new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.scoring_x + 0.35,AutoConstants.scoring2_y,Rotation2d.fromDegrees(180)),0.5),
            //     new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist)           
            // );
            
            case AUTO_RED_LEFT:
                return Commands.sequence(
                    new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(AutoConstants.starting_x, AutoConstants.community_rightCone_y, //-0.55
                    new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position               
                    new SetLiftAuto(m_robotContainer.m_Lift),// moves )arm out from starting position 
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,3,Rotation2d.fromDegrees(0)),1.25),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_rightCone_y,Rotation2d.fromDegrees(0)),1.25).alongWith(
                        new ArmWristAutoFloorCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Outtake)),// arm and wrist to floor cone intake position
                    new IntakeMoveStop(m_robotContainer.m_Outtake).withTimeout(10),// moves to left cone
                    new WaitCommand(0.1),// get cone 
                    new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist),// arm and wrist comes home 
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_rightCone_y,Rotation2d.fromDegrees(0)),1.25),//turn to face grid
                    // new WaitCommand(0.1),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(3.5,AutoConstants.scoring2_y,Rotation2d.fromDegrees(0)),1), //move to community
                    new WaitCommand(0.5),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.scoring_x + 0.05,AutoConstants.scoring2_y,Rotation2d.fromDegrees(0)),0.5).alongWith(
                        new ArmWristScoreNoTurn(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Lift).withTimeout(3.5)
                    ), //move to right node
                    new OuttakeMoveStop(m_robotContainer.m_Outtake),
                    new WaitCommand(0.5),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.scoring_x + 0.35,AutoConstants.scoring2_y,Rotation2d.fromDegrees(0)),0.5),
                    new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist)           
                );
                
                
                // return Commands.sequence(                                                         //2.81
                // new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(AutoConstants.starting_x, AutoConstants.community_leftCone_y - 0.1, //-0.55
                //     new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position               
                //     new SetLiftAuto(m_robotContainer.m_Lift),// moves )arm out from starting position 
                //     new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,3,Rotation2d.fromDegrees(0)),1.251),
                //     // arm and wrist to floor cone intake position 
                //     new ArmWristAutoFloorCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Outtake).alongWith(
                //         new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_leftCone_y,Rotation2d.fromDegrees(0)),1.251)),
                //     new IntakeMoveStop(m_robotContainer.m_Outtake).withTimeout(10),// moves to left cone
                //     new WaitCommand(0.05),// get cone 
                //     new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist),
                //     new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(3.5,AutoConstants.scoring1_y - 0.2,Rotation2d.fromDegrees(0)),1).withTimeout(3.5),
                //     new WaitCommand(0.1),
                //     new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d((AutoConstants.scoring_x + 0.3),AutoConstants.scoring1_y,Rotation2d.fromDegrees(0)),0.5).andThen(
                //         new ArmWristScoreNoTurn(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Lift).withTimeout(3.5)),
                //         new OuttakeMoveStop(m_robotContainer.m_Outtake),                
                //     new WaitCommand(0.1),
                //         new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist)
                        
                // );
                 

                
            case AUTO_JUST_CUBE:
                return Commands.sequence(
                    new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(0,0, //-0.55
                    new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position               
                    new SetLiftAuto(m_robotContainer.m_Lift)
                );
            
            case AUTO_CONE_RED_RIGHT_CRAZYARM: //arm 360
            return Commands.sequence(                                                         //2.81
            new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(AutoConstants.starting_x, AutoConstants.community_leftCone_y - 0.1, //-0.55
                new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position               
                new SetLiftAuto(m_robotContainer.m_Lift),// moves )arm out from starting position 
                new ArmWristAutoFloorCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Outtake).alongWith(
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_leftCone_y,Rotation2d.fromDegrees(0)),1.251).raceWith(
                    new IntakeMoveStop(m_robotContainer.m_Outtake).withTimeout(10))// moves to right cone
                ),// arm and wrist to floor cone intake position 
                new WaitCommand(0.05),// get cone 
                new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist),
                new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(3.5,AutoConstants.scoring1_y - 0.2,Rotation2d.fromDegrees(0)),1).withTimeout(3.5),
                new WaitCommand(0.1),
                new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d((AutoConstants.scoring_x + 0.3),AutoConstants.scoring1_y,Rotation2d.fromDegrees(0)),0.5).alongWith(
                    new ArmWristScoreNoTurn(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Lift).withTimeout(3.5)),
                new OuttakeMoveStop(m_robotContainer.m_Outtake),
                new WaitCommand(0.1),
                    new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist)
                );
                
                
                case AUTO_CONE_BLUE_LEFT_CRAZYARM: 
                return Commands.sequence(                                                         //2.81
                new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(AutoConstants.starting_x, AutoConstants.community_leftCone_y - 0.1, //-0.55
                    new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position               
                    new SetLiftAuto(m_robotContainer.m_Lift),// moves )arm out from starting position 
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,3,Rotation2d.fromDegrees(0)),1.251),
                    // arm and wrist to floor cone intake position 
                    new ArmWristAutoFloorCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Outtake).alongWith(
                        new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(AutoConstants.cone_x,AutoConstants.community_leftCone_y,Rotation2d.fromDegrees(0)),1.251)),
                    new IntakeMoveStop(m_robotContainer.m_Outtake).withTimeout(10),// moves to left cone
                    new WaitCommand(0.05),// get cone 
                    new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(3.5,AutoConstants.scoring1_y - 0.2,Rotation2d.fromDegrees(0)),1).withTimeout(3.5),
                    new WaitCommand(0.1),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d((AutoConstants.scoring_x + 0.3),AutoConstants.scoring1_y,Rotation2d.fromDegrees(0)),0.5).andThen(
                        new ArmWristScoreNoTurn(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Lift).withTimeout(3.5)),
                        new OuttakeMoveStop(m_robotContainer.m_Outtake),                
                    new WaitCommand(0.1),
                        new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist)
                        
                );
    


            case AUTO_DRIVE_TEST:
                return Commands.sequence( 
                new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(0, 0, //flush along edge/corner of line
                    new Rotation2d(0))), m_robotContainer.swerveDrive),
                new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(0,1,Rotation2d.fromDegrees(0)),1),
                new PrintCommand("PMT1 ended"),
                new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(1,1,Rotation2d.fromDegrees(0)),1),
                new PrintCommand("PMT2 ended"),
                new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(1,0,Rotation2d.fromDegrees(0)),1),
                new PrintCommand("PMT3 ended"),
                new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(0,0,Rotation2d.fromDegrees(0)),1),
                new PrintCommand("PMT4 ended")
                );
            
            case AUTO_SCORE_TEST:
            return Commands.sequence(
                new SetLiftAuto(m_robotContainer.m_Lift),
                new ArmWristScoreHighCone(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Lift).withTimeout(3.5),
                new OuttakeMoveStop(m_robotContainer.m_Outtake),
                new AutoHome(m_robotContainer.m_Lift, m_robotContainer.m_Arm, m_robotContainer.m_Wrist)
                // new ArmWristAutoGo(m_robotContainer.m_Arm, m_robotContainer.m_Wrist, m_robotContainer.m_Outtake)
                // new ArmWristAutoHome(m_robotContainer.m_Arm, m_robotContainer.m_Wrist)
            );

            case AUTO_JUST_BALANCE:
            return Commands.sequence(
            new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(0, 0, //-0.55
                    new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position
                    new SetLiftAuto(m_robotContainer.m_Lift),           
                    //new SetLiftAuto(m_robotContainer.m_Lift),// moves )arm out from starting position 
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(4.5,0,Rotation2d.fromDegrees(0)),0.75),
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(3,0,Rotation2d.fromDegrees(0)),0.75),
                    new AutonBalance(m_robotContainer.swerveDrive),
                    new PrintCommand("balanced")
            );

            case AUTO_BALANCE_NO_MOBILITY:
            return Commands.sequence(
            new InstantCommand(() -> m_robotContainer.swerveDrive.setSwervePosition(new Pose2d(0, 0, //-0.55
                    new Rotation2d(0))), m_robotContainer.swerveDrive), //set starting position
                    new SetLiftAuto(m_robotContainer.m_Lift),           
                    //new SetLiftAuto(m_robotContainer.m_Lift),// moves )arm out from starting position 
                    new ProfiledMoveTo(m_robotContainer.swerveDrive, new Pose2d(3,0,Rotation2d.fromDegrees(0)),0.75),
                    new AutonBalance(m_robotContainer.swerveDrive),
                    new PrintCommand("balanced")
            );

            default:
            return Commands.none();
        }
    }
}


