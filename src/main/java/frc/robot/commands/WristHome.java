// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class WristHome extends CommandBase {
  Wrist m_Wrist;
  /** Creates a new WristHome. */
  public WristHome(Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Wrist = wrist;
    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Wrist.setWrist(0, 0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    if (m_Wrist.getPosition() <= 15){
      return true;
    } else {
      return false;
    }
  }
}
