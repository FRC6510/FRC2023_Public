// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift.LiftReal;

public class LiftGo extends CommandBase {
  private LiftReal m_Lift;
  double m_Counts;
  double m_Speed;
  /** Creates a new Movelift. */
  public LiftGo(LiftReal lift, double counts, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    // this.m_Lift = new LiftReal();
    m_Lift = lift;
    m_Counts = counts;
    m_Speed = speed;
    addRequirements(m_Lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Lift.setLift(m_Counts, m_Speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    if (m_Lift.getPosition() >= (m_Counts - 100)){
      return true;
    } else {
      return false;
    }
  }
}
