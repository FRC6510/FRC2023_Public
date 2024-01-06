// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.OuttakeReal;
import frc.robot.commands.IntakeGo;
import frc.robot.commands.OuttakeGo;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeMoveStop extends SequentialCommandGroup {
  /** Creates a new IntakeMoveStop. */
  public IntakeMoveStop(OuttakeReal outtake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeGo(outtake, 7.5,0.9)
      // new IntakeStop(outtake)
    );
  }
}
