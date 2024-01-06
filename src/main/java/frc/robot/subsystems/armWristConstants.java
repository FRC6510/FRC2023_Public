// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class armWristConstants {

public static final double
    arm_kP = 0.5, //0.0005
    arm_kI = 0,//0,
    arm_kD = 0.3, //0.0003
    arm_kF = (0.5 * 1023) / 10000;//TODO 0.25 is the speed for now

public static final double
    wrist_kP = 0.5, //0.0005
    wrist_kI = 0,
    wrist_kD = 0; //0.0003
}
