// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import frc.robot.commands.Drivetrain.Profiled2DHeading;

/** Add your docs here. */
public class DrivetrainConstants {
public static final int 
    LF_DRIVE_MOTOR_PORT = 7,
    LB_DRIVE_MOTOR_PORT = 13,
    RF_DRIVE_MOTOR_PORT = 16,
    RB_DRIVE_MOTOR_PORT = 9;

public static final int
    LF_STEER_MOTOR_PORT = 8,
    LB_STEER_MOTOR_PORT = 11, 
    RF_STEER_MOTOR_PORT = 6,
    RB_STEER_MOTOR_PORT = 14; 

public static final int
    LF_ANGLE_ENCODER_PORT = 15,
    LB_ANGLE_ENCODER_PORT = 9,
    RB_ANGLE_ENCODER_PORT = 6,
    RF_ANGLE_ENCODER_PORT = 13;

public static final double
    ROBOTMAXSPEED = 4;//3.6

public static final double
    rotational_kP = 0.013, //0.019
    rotational_kI = 0,
    rotational_kD = 0; //0.0001

public static final double 
    linear_kP = 0.06,
    linear_kI = 0,
    linear_kD = 3;

public static final double
    BL_OFFSET = 59.5898438 - -179.736,
    BR_OFFSET = 149.677734 - 0.352,
    FL_OFFSET = 11.953125 - 18.896,
    FR_OFFSET = 104.326172 - 176.396;

public static final double
    theta_kP = 0.1,
    theta_kI = 0,
    theta_kD = 0;

public static final double
    linear_profiled_kP = 0.1,
    linear_profiled_kI = 0,
    linear_profiled_kD = 0;

public static final double //stolen from 2022
    MAX_SPEED_METRES_PER_SECOND = 3.5, //0.8 - 93
    MAX_ACCELERATION_METRES_PER_SECOND_SQUARED = MAX_SPEED_METRES_PER_SECOND,
    MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 2*Math.PI,
    MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_SQUARED = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,

    GEAR_RATIO = 6.75, //one rotation of wheel from 6.75 shaft rotations
    CANCODER_RAW_TICKS_PER_180 = 2048,
    WHEEL_RADIUS = 0.0508,
    WHEEL_CIRCUMFERENCE = 0.1016*Math.PI, //2*(WHEEL_RADIUS*25.4)*Math.PI, //TODO is wrong... recalculate with new robot
    HALF_ROTATION = CANCODER_RAW_TICKS_PER_180 / 180.0,
    TALONFX_RESOLUTION = 2048,

    TICKS_PER_WHEEL_ROTATION = TALONFX_RESOLUTION * GEAR_RATIO,

    // ENCODER_TO_ANGLE_MULTIPLIER = (2*Math.PI)/TICKS_PER_WHEEL_ROTATION,
    // DISTANCE_MULTIPLIER = WHEEL_RADIUS * ENCODER_TO_ANGLE_MULTIPLIER,
    DISTANCE_MULTIPLIER = WHEEL_CIRCUMFERENCE/TICKS_PER_WHEEL_ROTATION,

    OMEGA = ((2*Math.PI)/TICKS_PER_WHEEL_ROTATION), //conversion from radians per 100ms to 1000ms (1s) //removed TEN multiplier
    VELOCITY_MULTIPLIER = OMEGA * 2 * WHEEL_RADIUS;

    public static final Pose2d
    endTolerance  = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(3)); // ASAPH 0.05 0.05 3

}

