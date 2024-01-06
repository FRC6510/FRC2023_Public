// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmHome;
import frc.robot.commands.LiftHome;
import frc.robot.commands.WristHome;
import frc.robot.subsystems.Lift.LiftReal;
import frc.robot.subsystems.ArmReal;
import frc.robot.subsystems.Wrist;
// import frc.robot.subsystems.OuttakeReal;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoHome extends SequentialCommandGroup {
  /** Creates a new ArmWristAutoHome. */
  public AutoHome(LiftReal lift, ArmReal arm, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WristHome(wrist).alongWith(
        new ArmHome(arm),
        new LiftHome(lift)
      )
    );
  }
}
