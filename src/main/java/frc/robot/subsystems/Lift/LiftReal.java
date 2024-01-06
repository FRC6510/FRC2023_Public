// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Lift;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class LiftReal extends SubsystemBase {

  public liftConstants m_LiftConstants;
  //public liftGains m_liftGains;
  private static WPI_TalonFX liftMotor;

  public double liftMax = 113000; //113568
  public double liftMin = 233;

  public double testLiftPosition = liftMax / 3;

  static double liftTargetPosition;

  /** Creates a new LiftReal. */
  public LiftReal() {
    liftMotor = new WPI_TalonFX(9, "CANivore");//Todo MAKE SURE THE LIFT IS GOING IN THE RIGHT DIRECTION 
    initLift();
    liftMotor.setSelectedSensorPosition(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    printLiftPosition();
  }

  public double getPosition(){
    return liftMotor.getSelectedSensorPosition();
  }

  public void setLift(double position, double speed){
    liftMotor.configPeakOutputForward(speed, liftConstants.kTimeoutMs);
    liftMotor.configPeakOutputReverse(-speed, liftConstants.kTimeoutMs);

    liftTargetPosition = position;
    liftMotor.set(ControlMode.Position, liftTargetPosition);
  }

  public void setLiftMax(){
    liftMotor.set(ControlMode.Position, liftMax);
  }

  public void setLiftMin(){
    liftMotor.set(ControlMode.Position, liftMin);
  }

  public void printLiftPosition(){
    SmartDashboard.putNumber("Lift position", liftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Lift speed", liftMotor.getMotorOutputPercent());
  }

  public void initLift(){//TODO change this - change this by referencing the FRC 2022 github
    liftMotor.configFactoryDefault();//TODO set inverted for lift -> make sure it's right on phoenix tuner first 
    // liftMotor.setInverted(false);
    liftMotor.setNeutralMode(NeutralMode.Coast);

    liftMotor.configNominalOutputForward(0, liftConstants.kTimeoutMs);
		liftMotor.configNominalOutputReverse(0, liftConstants.kTimeoutMs);
		liftMotor.configPeakOutputForward(0.25, liftConstants.kTimeoutMs);
    liftMotor.configPeakOutputReverse(-0.25, liftConstants.kTimeoutMs);

    liftMotor.config_kP(liftConstants.kPIDLoopIdx, 0.5, liftConstants.kTimeoutMs);//liftConstants.kGains_Velocit.kP, liftConstants.kTimeoutMs);
    liftMotor.config_kD(liftConstants.kPIDLoopIdx, 0, liftConstants.kTimeoutMs);//liftConstants.kGains_Velocit.kD, liftConstants.kTimeoutMs);
    liftMotor.config_kI(liftConstants.kPIDLoopIdx, 0, liftConstants.kTimeoutMs);//liftConstants.kGains_Velocit.kI, liftConstants.kTimeoutMs);

  }

}
