// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Drivetrain.ProfiledMoveTo;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

public class auto_test extends CommandBase {
  /** Creates a new motion_profiling_test_1. */
  public auto_test(SwerveDrive drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
     new ProfiledMoveTo(drivetrain,new Pose2d(1, 0,Rotation2d.fromDegrees(0)),1);

    }
  }

