// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmGo;
import frc.robot.commands.LiftGo;
import frc.robot.commands.WristGo;
import frc.robot.subsystems.ArmReal;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Lift.LiftReal;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmWristScoreNoTurn extends SequentialCommandGroup {
  /** Creates a new ArmWristScoreHighCone. */
  public ArmWristScoreNoTurn(ArmReal arm, Wrist wrist, LiftReal lift) {
    double armMidConeNoTurn = 224844;
    double wristMidConeNoTurn = 38303;
    double liftMidCone = 42399 / 3;
    // double armHighCone = 115500;// 99884
    // double wristHighCone = 83900 + 5500; //83900 + 4500
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new LiftGo(lift, liftMidCone, 0.25),
      // new PrintCommand("Lift finished"),
      // new ArmGo(arm, armMidConeNoTurn, 0.6).andThen(
      //   new WristGo(wrist, 0, 0.6) 
      // ),
      new ArmGo(arm, armMidConeNoTurn, 0.45).andThen(
        new WristGo(wrist, wristMidConeNoTurn, 0.45) 
      )

      // new PrintCommand("Wrist finished")
    );
  }
}
