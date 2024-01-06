// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmReal;

public class ArmGo extends CommandBase {
  ArmReal m_Arm;
  double m_Count;
  double m_Speed;
    /** Creates a new WristGo. */
  public ArmGo(ArmReal arm, double counts, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = arm;
    m_Count = counts;
    m_Speed = speed;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setArm(m_Count, m_Speed);
    // m_Arm.setArm(11865.0,0.5); -> this is for intaking cones
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_Arm.setArm(m_Count, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Arm.getPosition() >= (m_Count - 100)){
      return true;
    } else {
      return false;
    }
  }
}
