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
public class ArmWristScoreHighCone extends SequentialCommandGroup {
  /** Creates a new ArmWristScoreHighCone. */
  public ArmWristScoreHighCone(ArmReal arm, Wrist wrist, LiftReal lift) {
    double liftHighCone = 42399 / 3;
    double armHighCone = 114461 - 1050;// 99884
    double wristHighCone = 83900 + 1350; //83900 + 4500
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LiftGo(lift, liftHighCone, 0.25),
      // new PrintCommand("Lift finished"),
      new ArmGo(arm, armHighCone, 0.6).alongWith(
        new WristGo(wrist, wristHighCone, 0.6) 
      )
      // new PrintCommand("Wrist finished")
    );
  }
}
