// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.revrobotics.AbsoluteEncoder;

// import frc.robot.subsystems.armWristConstants;

public class Wrist extends SubsystemBase {

  static WPI_TalonFX WristMotor;

  static DutyCycleEncoder wristAbEncoder;

  static double gearRatio = 10 * 2 * (3 + 1/3);

  static double countsPerRev = 2048;
  double offset;

  static double countsPerWristRev = gearRatio * countsPerRev;
  static double wristTargetPosition;

  /** Creates a new Wrist. */
  public Wrist(){
    WristMotor = new WPI_TalonFX(30, "CANivore");
    WristMotor.setNeutralMode(NeutralMode.Brake);
    WristMotor.setInverted(true);
    
    WristMotor.configFactoryDefault();

    WristMotor.config_kP(0, armWristConstants.wrist_kP);
    WristMotor.config_kI(0, armWristConstants.wrist_kI);
    WristMotor.config_kD(0, armWristConstants.arm_kD);

    WristMotor.configPeakOutputForward(0.6);
    WristMotor.configPeakOutputReverse(-0.6);
    WristMotor.setSelectedSensorPosition(0);//figure out how to set the starting wrist encoder values using the absolute  encoder 

    // wristAbEncoder = new DutyCycleEncoder(1);
    // wristAbsEncoder.set
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    printWristInfo();
  }

  public void PrintEncoderPosition (){
    double position = WristMotor.getSelectedSensorPosition();
    // double absoluteEncoderPosition = wristAbEncoder.getAbsolutePosition();
    // SmartDashboard.putNumber("Wrist encoder position", position);
    // SmartDashboard.putNumber("Wrist absolute encoder", absoluteEncoderPosition);

    
  }

  public void setWrist(double wristTargetPosition, double speed) {
    //This method was tested and the shooter did not spin when the method was called.
    // Can you see how to fix i
    //wristTargetPosition = angleToCounts(angle); <-- only when receiving angles
    // WristMotor.set(0.3);
    // if (wristAbEncoder.getAbsolutePosition() <= 0.2625 && wristAbEncoder.getAbsolutePosition() >= 0){
    //   WristMotor.set(TalonFXControlMode.Position, wristTargetPosition); //
    // }
    WristMotor.configPeakOutputForward(speed);
    WristMotor.configPeakOutputReverse(-speed);
    WristMotor.set(TalonFXControlMode.Position, wristTargetPosition); //
  }

  public double getPosition(){
    return WristMotor.getSelectedSensorPosition();
  }


  public static double angleToCounts(double angle){
    // May have to find objective angle from angle relative to arm? 
    double countConversion = angle * (countsPerWristRev / 360);
    return countConversion;

    /*
    * 360 degr. = countsPerArmRev (1 revolution)
    * 1 degr. = countsPerArmRev / 360
    */
  }

  public static double countsToAngle(double counts){
    double angleConversion = (counts / countsPerRev) * 360;
    return angleConversion;
  }

  public void changeToCoast(){
    WristMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void changeToBrake(){
    WristMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void printWristInfo(){
    PrintEncoderPosition();
    SmartDashboard.putNumber("Wrist target position", wristTargetPosition);
    SmartDashboard.putNumber("Wrist counts", WristMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Wrist angle", countsToAngle(WristMotor.getSelectedSensorPosition()));
  }
}