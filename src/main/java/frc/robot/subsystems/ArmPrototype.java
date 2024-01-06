// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ArmPrototype extends SubsystemBase {
  //constants
   double gearRatio = 161.62;
   double countsPerRev = 2048;
   double offset;
   double angle;

   double countsPerTurn = gearRatio * countsPerRev;

  TalonFX arm;//TODO: change where necessary
  
  /** Creates a new ArmPrototype. */
  public ArmPrototype() {
    arm = new TalonFX(2);
    arm.setNeutralMode(NeutralMode.Coast);
    arm.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double encoderReset(){
    offset = arm.getSelectedSensorPosition();
    return offset;
  }


  public void getEncoderPosition(){
    // SmartDashboard.putNumber("Arm position: ", arm.getSelectedSensorPosition());
  }

  public void moveToPosition(double position, double speed){
    arm.set(TalonFXControlMode.Position, position * 10000);
    //arm.set(TalonFXControlMode.PercentOutput, speed);

    
    if (arm.getSelectedSensorPosition() > 8.050 * 10000){
      stopArm();
    }
    
  }

  public void moveArm(double speed){
    arm.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void stopArm(){
    arm.set(TalonFXControlMode.PercentOutput, 0);
  }

  /*
   * Arm positions:
   * Down to the ground: -0.424
   * Up (parallel with the ground): 8.050 
   */

}
