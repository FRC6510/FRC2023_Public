// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule{

  public WPI_TalonFX motorSteer;
  public WPI_TalonFX motorDrive;
  public CANCoder encoder;
  public PIDController pidDirectionController;
  double angleOutput;

  public SwerveModule(double kP, double kI, double kD, WPI_TalonFX motorSteer, WPI_TalonFX motorDrive, CANCoder encoder) {
    this.motorSteer = motorSteer;
    this.motorDrive = motorDrive;
    this.encoder = encoder;
    pidDirectionController = new PIDController(kP, kI, kD);
    pidDirectionController.enableContinuousInput(-180, 180);
    motorDrive.setNeutralMode(NeutralMode.Brake);
  }


  public void updatePID(){
    angleOutput = pidDirectionController.calculate(encoder.getAbsolutePosition());
    
    motorSteer.set(ControlMode.PercentOutput, angleOutput);
    //make it actually change speeds
  }

  public void setDesiredState(SwerveModuleState desiredState){
    //the desired state is ok. But it's going to the steer motors instead...
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(encoder.getAbsolutePosition()));
    //double speed = 1; //desiredState.speedMetersPerSecond;
    pidDirectionController.setSetpoint(desiredState.angle.getDegrees());

    // SmartDashboard.putNumber("desiredState", desiredState.speedMetersPerSecond);
    // SmartDashboard.putNumber("desiredState encoder ticks", desiredState.speedMetersPerSecond/3.5);

    //motorDrive.set(ControlMode.Velocity, convertToWheelEncoderTicks(desiredState.speedMetersPerSecond));
    motorDrive.set(ControlMode.PercentOutput, desiredState.speedMetersPerSecond/3.5);
    //motorSteer.set(ControlMode.PercentOutput, angleOutput*0.05);

    // SmartDashboard.putNumber("desired state angle", desiredState.angle.getDegrees());
  }

  public void setDesiredStateVelocity(SwerveModuleState desiredState){
    //the desired state is ok. But it's going to the steer motors instead...
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(encoder.getAbsolutePosition()));
    //double speed = 1; //desiredState.speedMetersPerSecond;
    pidDirectionController.setSetpoint(desiredState.angle.getDegrees());
    //double angleOutput = pidDirectionController.calculate(encoder.getAbsolutePosition(), desiredState.angle.getDegrees());

    // SmartDashboard.putNumber("desiredState", desiredState.speedMetersPerSecond);
    // SmartDashboard.putNumber("desiredState encoder ticks", desiredState.speedMetersPerSecond/3.5);

    //motorDrive.set(ControlMode.Velocity, convertToWheelEncoderTicks(desiredState.speedMetersPerSecond));
    motorDrive.set(ControlMode.Velocity, convertToWheelEncoderTicksPerHundredMilliseconds(desiredState.speedMetersPerSecond));
    //motorSteer.set(ControlMode.PercentOutput, angleOutput*0.05);

    // SmartDashboard.putNumber("desired state angle", desiredState.angle.getDegrees());
  }

  public double convertToWheelEncoderTicksPerHundredMilliseconds(double wheelVelocity){
    return wheelVelocity/DrivetrainConstants.VELOCITY_MULTIPLIER;
  }

  public void setSteerPower(double steerPower){
    motorSteer.set(steerPower);
  }

  //m/s chassis speeds to velocity because motion profiling needs velocity to be accurate

  public void setDrivePower(double drivePower){
    motorDrive.set(drivePower);
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
      (motorDrive.getSelectedSensorPosition() * DrivetrainConstants.DISTANCE_MULTIPLIER), 
      Rotation2d.fromDegrees(encoder.getAbsolutePosition())
    );
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

  //need to go from m/s to velocity in encoder ticks
