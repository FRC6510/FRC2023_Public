// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Main;
import frc.robot.commands.SimplerMainStateMachine;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private double x = 0;
  private double y = 0;
  private double area = 0;
  private static boolean valid = false;

  public final int leftCubeNodePipeline = 0;
  public final int centreCubeNodePipeline = 1;
  public final int rightCubeNodePipeline = 2;
  public final int doubleSubstationPipeline = 3;
  public final int retroflectiveTape = 4;// usage is yet to be confirmed

  static double targetPipeline;

  public Limelight() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = table.getEntry("tx").getDouble(0.0);
    y = table.getEntry("ty").getDouble(0.0);
    area = table.getEntry("ta").getDouble(0.0);
    if(table.getEntry("tv").getDouble(0) == 0){
      valid = false;
    }
    else{
      valid = true;
    }

    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);
    // SmartDashboard.putNumber("Target Pipeline", targetPipeline);

  }


  public void setPipeline(int targetPipeline){
    if (!(targetPipeline<10 && targetPipeline>-1)){
        return;
    }
    table.getEntry("pipeline").setNumber(targetPipeline);
    Limelight.targetPipeline = targetPipeline;
  }

//   public void increasePipeline(){
//     if (!(targetPipeline<10 && targetPipeline>-1)){
//         return;
//     }
//     table.getEntry("pipeline").setNumber(targetPipeline++);

// }
// public void decreasePipeline(){
//   if (!(targetPipeline<10 && targetPipeline>-1)){
//       return;
//   }
//   table.getEntry("pipeline").setNumber(targetPipeline--);

// }

public void driverCamera(){
  table.getEntry("camMode").setNumber(1);

}

public void visionProcesser(){
  table.getEntry("camMode").setNumber(0);
}

public double getX() {
    return x;
}
public double getY() {
    return y;
}
public double getArea() {
    return area;
}
public boolean getValid(){
  return valid;
}

public void chooseSide(){

  switch(SimplerMainStateMachine.m_gamepiece){
    case CONE:
    if(x <= 0){// if the value of x is less than 1, it means the camera is on the left of the tag 
      setPipeline(leftCubeNodePipeline);
    }

     else if(x > 0){// if the value of x is greater than or equals to 1, it means that the camera is on the right of the tag 
      setPipeline(rightCubeNodePipeline);
    } else {
      return;
    }
      break;

    case CUBE:
    setPipeline(centreCubeNodePipeline);
      break;
  }
  
}

}
