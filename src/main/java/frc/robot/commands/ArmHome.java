// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmReal;

public class ArmHome extends CommandBase {
  ArmReal m_Arm;
  /** Creates a new WristHome. */
  public ArmHome(ArmReal arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = arm;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setArm(0, 0.35);// to be safe 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Arm.getPosition() <= 10){
      return true;
    } else {
      return false;
    }
  }
}
