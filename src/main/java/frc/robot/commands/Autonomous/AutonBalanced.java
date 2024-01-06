// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

public class AutonBalanced extends CommandBase {
  private SwerveDrive m_swerveDrive;
  double xSpeed;
  double ySpeed;
  double angularVelocity;
  double x_offset_angle = 3;
  double y_offset_angle = 3;
  Timer timer;


  /** Creates a new AutonBalanceReal. */
  public AutonBalanced(SwerveDrive m_swerveDrive) {
    this.m_swerveDrive = m_swerveDrive;
    timer = new Timer();

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_swerveDrive.setSteerPosition(angularVelocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

      m_swerveDrive.backLeft.motorSteer.set(ControlMode.Position,135);
      m_swerveDrive.frontLeft.motorSteer.set(ControlMode.Position,45);
      m_swerveDrive.backRight.motorSteer.set(ControlMode.Position,45);
      m_swerveDrive.frontRight.motorSteer.set(ControlMode.Position,135);

      xSpeed = 0;
      ySpeed = 0;
      angularVelocity = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean filteredFinish = false;
    if (!(x_offset_angle < 2 && x_offset_angle > -2 && y_offset_angle < 2 && y_offset_angle > -2)){
      timer.reset();
    }
    else if (timer.advanceIfElapsed(0.2)){
      filteredFinish = true;
    }
    return filteredFinish;
  }
}
