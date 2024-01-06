// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.commands.SimplerMainStateMachine;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.RainbowAnimation;
// import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
// import com.ctre.phoenix.led.ColorFlowAnimation;


// public class LEDs extends SubsystemBase {
//   CANdle m_Candle;
//   public boolean wantpurple;
//   public boolean wantyellow;
//   RainbowAnimation m_RainbowAnimation = new RainbowAnimation();
//   ColorFlowAnimation yellowAnimate = new ColorFlowAnimation(252, 229, 28, 0, 0.05, 8, Direction.Backward);
//   ColorFlowAnimation purpleAnimate = new ColorFlowAnimation(94, 12, 166, 0, 0.05, 8, Direction.Backward);

//   ColorFlowAnimation fastyellowAnimate = new ColorFlowAnimation(252, 229, 28, 0, 0.55, 8, Direction.Backward);
//   ColorFlowAnimation fastpurpleAnimate = new ColorFlowAnimation(94, 12, 166, 0, 0.55, 8, Direction.Backward);

//   /** Creates a new LEDs. */
//   public LEDs() {
//     m_Candle = new CANdle(1);//TODO add
//     wantpurple = false;
//     wantyellow = false;
    
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   // public void animateRainbow(){
//   //   m_Candle.animate(m_RainbowAnimation);
//   // }
//   public void animateRainbow(){
//     m_Candle.animate(m_RainbowAnimation);
//   }

//   public void turnYellow(){
//     m_Candle.animate(yellowAnimate);
//   }

//   public void turnPurple(){
//     m_Candle.animate(purpleAnimate);

//     //m_Candle.setLEDs(94, 12, 166);
//     //mCaNdle.setLEDs(0, 0, 0, 0, 0, 0);
//   }

//   public void yellowFast(){
//     m_Candle.animate(fastyellowAnimate);
//   }

//   public void purpleFast(){
//     m_Candle.animate(fastpurpleAnimate);
//   }

//   // public void resetAnimation(){
//   //   m_Candle.clearAnimation(0);
//   // }

//   // public void resetLEDS(){
//   //   m_Candle.setLEDs(0, 0, 0);
//   // }
//   public void resetAnimation(){
//     m_Candle.clearAnimation(0);
//   }


//   public void changeColour(XboxController controller){
//     if (controller.getRightTriggerAxis() > 0.1){//controller.getRawButtonPressed(10)
//       wantpurple = true;
//       wantyellow = false;
//     } else if (controller.getLeftTriggerAxis() > 0.1){//controller.getRawButtonPressed(9)
//       wantyellow = true;
//       wantpurple = false;
//       // SmartDashboard.putString("Want yellow", "getting yellow");
//     } else {
//       wantpurple = false;
//       wantyellow = false;
//     }
//     // }

//   }

//     // SmartDashboard.putString("Changing colour", "Changing");
//   }


