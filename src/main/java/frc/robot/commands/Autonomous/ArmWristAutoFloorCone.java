// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Lift.LiftReal;
import frc.robot.subsystems.ArmReal;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.OuttakeReal;
import frc.robot.commands.ArmGo;
import frc.robot.commands.WristGo;
import frc.robot.commands.IntakeGo;
import frc.robot.commands.OuttakeGo;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmWristAutoFloorCone extends SequentialCommandGroup {
  /** Creates a new ArmWristAuto. */
  public ArmWristAutoFloorCone(ArmReal arm, Wrist wrist, OuttakeReal outtake) {
    double armConeFloor = 11865;
    double wristConeFloor = 37742;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmGo(arm, armConeFloor, 0.5),
      new WristGo(wrist, wristConeFloor, 0.6)
      // new IntakeGo(outtake).withTimeout(1)
    );
  }
}
