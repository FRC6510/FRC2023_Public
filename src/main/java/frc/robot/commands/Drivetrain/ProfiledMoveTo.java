// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms 
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

public class ProfiledMoveTo extends CommandBase {
  double maxA = 2; // m/s^s -> resting speed to max speed difference over time. Constant.
  double current_length = 0; // idk?????? no.
  double next_velocity = 0;
  double next_velocity_heading = 0;
  double m_goalLength;
  double m_goalHeading;
  double angle_to_target;
  double max_velocity;
  SwerveDrive m_drivetrain;
  Pose2d m_targetCoords;
  Pose2d m_initialCoords;
  Pose2d nextCoordinates;
  double tolerance;
  // HolonomicDriveController m_controller;
  ProfiledPIDController m_linearPIDController;
  ProfiledPIDController m_thetaController;
  boolean hasEnded = false;

  /** Creates a new ProfiledMoveTo. */
  public ProfiledMoveTo(SwerveDrive drivetrain, Pose2d targetCoords, double velocity) {
    m_thetaController = new ProfiledPIDController(1.5, 0, 0,
        new TrapezoidProfile.Constraints(4, 1.5));

    m_linearPIDController = new ProfiledPIDController(0.5, 0, 0, // 0.7,0,0
        new TrapezoidProfile.Constraints(velocity, velocity * 0.5)); // *0.67// from 18 hrs ago: 1.5
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // find length and heading
    m_targetCoords = targetCoords;
    max_velocity = velocity;
    m_drivetrain = drivetrain;
    // m_controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialCoords = m_drivetrain.m_odometry.getPoseMeters();
    m_linearPIDController.reset(0);
    m_thetaController.reset(m_initialCoords.getRotation().getRadians());

    m_goalLength = m_targetCoords.getTranslation().getDistance(m_initialCoords.getTranslation());
    m_goalHeading = m_targetCoords.getRotation().getRadians();

    m_linearPIDController.setGoal(new State(m_goalLength, 0));
    m_thetaController.setGoal(new State(m_goalHeading, 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("target x", m_targetCoords.getX());
    // SmartDashboard.putNumber("target y", m_targetCoords.getY());
    // SmartDashboard.putNumber("target heading", m_targetCoords.getRotation().getDegrees());


    // SmartDashboard.putNumber("heading error", m_goalHeading - angle_to_target);
    double distance_travelled = m_drivetrain.m_odometry.getPoseMeters().getTranslation()
        .getDistance(m_initialCoords.getTranslation());

    double errorVelocityCompensation = m_linearPIDController.calculate(distance_travelled);

    // double currentTargetAlongTrajectory =
    // m_linearPIDController.getSetpoint().position;
    // pythag distance between here and end goal

    double errorVelocityCompensationHeading = m_thetaController
        .calculate(m_drivetrain.m_odometry.getPoseMeters().getRotation().getRadians());

    // double currentTargetAlongTrajectoryHeading =
    // m_thetaController.getSetpoint().position;

    angle_to_target = atanCalculator(m_drivetrain.m_odometry.getPoseMeters()); //not to be used for heading
    // SmartDashboard.putNumber("current heading atan", angle_to_target);
    /*
     * Does it matter that atan2 outputs radians????
     * should it be x/y????
     */

    // nextCoordinates = new Pose2d(currentTargetAlongTrajectory *
    // Math.cos(current_heading), currentTargetAlongTrajectory *
    // Math.sin(current_heading), new
    // Rotation2d(currentTargetAlongTrajectoryHeading));

    next_velocity = m_linearPIDController.getSetpoint().velocity + errorVelocityCompensation;
    System.out.println(errorVelocityCompensationHeading);
    next_velocity_heading = m_thetaController.getSetpoint().velocity + errorVelocityCompensationHeading;

    // profilers gives position with velocity
    // at that position calculate error
    // where we want to be - where we are

    // set current velocity to be the setpoint velocity from the trapezoidal profile
    // the velocity of the robot should e going at the speed that the trapezoidal
    // state is telling it to go at
    // adjust based on distance

    m_drivetrain.drive(next_velocity * Math.cos(angle_to_target), next_velocity * Math.sin(angle_to_target),
        next_velocity_heading, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false);
    hasEnded = true;
    // SmartDashboard.putBoolean("has ended", hasEnded);
    System.out.println("end method" + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    // changed from currentlyTargetedCoodinates to current position, also wrapped in
    // Math.abs
    return Math.abs(
        m_targetCoords.getTranslation().getDistance(m_drivetrain.m_odometry.getPoseMeters().getTranslation())) < 0.1 &&
        Math.abs(m_goalHeading - m_drivetrain.m_odometry.getPoseMeters().getRotation().getRadians()) < 10 * (Math.PI/180);

    // heading error -> 1.46, 10 * PI/180 -> 0.174533

  }

  public double atanCalculator(Pose2d currentPosition) {
    return Math.atan2(m_targetCoords.getY() - currentPosition.getY(), m_targetCoords.getX() - currentPosition.getX());
  }

  private double findHeadingError(double currentHeading, double targetHeading) {
    currentHeading %= 360;
    targetHeading %= 360;
    if (currentHeading < 0) {
      currentHeading += 360;
    }
    if (targetHeading < 0) {
      targetHeading += 360;
    }

    double difference = currentHeading - targetHeading;
    if (difference > 180) {
      return (-360) + difference;
    } else if (difference < -180) {
      return 360 + difference;
    } else {
      return difference;
    }
  }
}