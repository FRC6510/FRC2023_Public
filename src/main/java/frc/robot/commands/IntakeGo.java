// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeReal;
import edu.wpi.first.wpilibj.Timer;

public class IntakeGo extends CommandBase {
  OuttakeReal m_Outtake;
  Timer m_Timer;
  double m_TimeOut;
  double m_speed;
  /** Creates a new IntakeGo. */
  public IntakeGo(OuttakeReal outtake, double timeOut, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    // m_Outtake = new OuttakeReal();
    m_Outtake = outtake;
    m_TimeOut = timeOut;
    m_Timer = new Timer();
    m_speed = speed;
    addRequirements(m_Outtake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Outtake.outtakeAutoSpin(m_speed);
    m_Timer.start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Outtake.outtakeAutoSpin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_Timer.get();
    if (m_Timer.get() > m_TimeOut){
      return true;
    } else {
      return false;
    }
  }
}
