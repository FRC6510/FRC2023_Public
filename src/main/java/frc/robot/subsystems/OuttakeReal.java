// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SimplerMainStateMachine;
import frc.robot.commands.SimplerMainStateMachine.gamepieces;
import frc.robot.commands.SimplerMainStateMachine.scoringPosition;
//import frc.robot.subsystems.LEDs;

import org.opencv.photo.MergeRobertson;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OuttakeReal extends SubsystemBase {
  public static CANSparkMax m_outtake;
  public boolean objectDetected = false;//static objects almost always need an initial value 
  private double countsperRev = 42; //TODO: measure 
  public double targetSpeed = 0;
  private double targetPosition = 0;
  public boolean outtaken;
  public boolean autoIsDone;
  public boolean hasYellow;
  public boolean hasPurple;
  private SparkMaxRelativeEncoder m_RelativeEncoder;
  Timer m_Timer = new Timer();

  /** Creates a new OuttakeRel. */
  public OuttakeReal() {
    m_outtake = new CANSparkMax(50, MotorType.kBrushless);
    m_outtake.setIdleMode(IdleMode.kBrake);
    m_outtake.setInverted(true);
    m_outtake.disableVoltageCompensation(); 

    outtaken = true;
    autoIsDone = false;
  }

  // public void updateObjectDetection(){

  //   double outtakePreviousChange = 0;
  //   double outtakePreviousPosition = 0;
  //   // double outtakePreviousSpeed = 0;
  //   double OuttakeCurrentPosition = m_outtake.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, countsperRev).getPosition();


  //   if (m_outtake.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, countsperRev).getPosition() != outtakePreviousPosition){
  //     m_Timer.reset();
  //   }

  //   double OuttakeChange = OuttakeCurrentPosition - outtakePreviousPosition;

  //   if ((OuttakeChange - outtakePreviousChange) < -0.1){//TODO measure the constant
  //     objectDetected = true;
  //   } else {
  //     objectDetected = false;
  //   }

  //   SmartDashboard.putNumber("Outtake change", OuttakeChange - outtakePreviousChange);
  //   SmartDashboard.putNumber("Outtake position", OuttakeCurrentPosition);
  //   SmartDashboard.putBoolean("Object detected", objectDetected);

  //   outtakePreviousPosition = OuttakeCurrentPosition;
  //   outtakePreviousChange = OuttakeChange;
  // }

  public void outtakeAutoSpin(double speed){
    m_Timer.reset();
    m_Timer.start();
    m_outtake.set(speed);
    m_Timer.get();
    if (m_Timer.get() >=3){
      m_outtake.set(speed * 0.1);
      autoIsDone = true;
    }
  }
//comment !!

  // public void intakeAdjustment(double speed, XboxController controller){
  //   if (controller.getRightBumper()){
  //     m_outtake.set(-speed);// outtake cone 
  //   } else if (controller.getLeftBumper()){
  //     m_outtake.set(speed);// intake cone 

  //   }
  // }

  public void spinIntake(double speed, XboxController controller){
    if (controller.getRightBumper()){
      m_Timer.reset();
      m_Timer.start();
      switch (SimplerMainStateMachine.m_gamepiece){
        case CUBE:// intake cube 
          m_outtake.set(-speed);
          outtaken = false;
        break;

        case CONE:// outtake cone 
          if (SimplerMainStateMachine.m_scoringPosition == scoringPosition.LOW_CONE){
            m_outtake.set(-speed * 0.5);
            outtaken = true;
          } else{
            m_outtake.set(-speed);
            outtaken = true;
          }
          // SimplerMainStateMachine.m_gamepiece = gamepieces.NONE;
        break;

        case NONE:
          m_outtake.set(0.1);
        break;

      }
    } else if (controller.getLeftBumper()){
      m_Timer.reset();
      m_Timer.start();
      switch (SimplerMainStateMachine.m_gamepiece){
        case CUBE:// outtake cube
          m_outtake.set(speed);
          outtaken = true;
          // SimplerMainStateMachine.m_gamepiece = gamepieces.NONE;
        break;

        case CONE:// intake cone 
          m_outtake.set(speed);
          outtaken = false;
        break;

        case NONE:
          m_outtake.set(0.1);
        break;
      }
    } else {
      switch (SimplerMainStateMachine.m_gamepiece){
        case CUBE:
          if (outtaken){
            m_outtake.set(0.1);// default now 
            hasPurple = false;
          } else {
            m_outtake.set(-0.2);
            hasPurple = true;
          }
        break;

        case CONE:
          if (outtaken){
            m_outtake.set(0.1);// default now 
            hasYellow = false;
          } else {
            m_outtake.set(0.2);// stability for driving fast 
            hasYellow = true;
          }
        break;

        case NONE:
          m_outtake.set(0.1);
          hasYellow = false;
          hasPurple = false;
        break;
    }
  }
}

public double returnTimer(){
  return m_Timer.get();
}

public void outtakeinfo(){
  SmartDashboard.putNumber("1 outtake encoder position", m_outtake.getEncoder(Type.kHallSensor, 42).getPosition());
  SmartDashboard.putNumber("1 outtake motor power", m_outtake.getOutputCurrent());
}

  /*
   * else if (!controller.getRightBumper() && !controller.getLeftBumper()) {
      switch (SimplerMainStateMachine.m_gamepiece){
        case CUBE:
        double intakePosition = m_outtake.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, countsperRev).getPosition();
        m_outtake.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, countsperRev).setPosition(intakePosition);
        break;

        case CONE:
          m_outtake.set(-speed);
        break;
   */
    

   
  // public void setPosition(double position){
  //   while (OuttakePosition < position){
  //     outtake.set(0.1);
  // }
  //   while (OuttakePosition > position){
  //     outtake.set(-0.1);
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateObjectDetection();
    outtakeinfo();
  }

 
}
