// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SwerveDrive extends SubsystemBase {

  public SwerveModule frontLeft;
  public SwerveModule frontRight;
  public SwerveModule backLeft;
  public SwerveModule backRight;

  public static boolean isFieldCentric = false;

  // simba things
  // public static WPI_Pigeon2 imu = new WPI_Pigeon2(0, "canivore");

  // static WPI_TalonFX motorBLSteer = new WPI_TalonFX(16, "canivore"); //15
  // static WPI_TalonFX motorBRSteer = new WPI_TalonFX(14, "canivore"); //15
  // static WPI_TalonFX motorFLSteer = new WPI_TalonFX(15, "canivore"); //15
  // static WPI_TalonFX motorFRSteer = new WPI_TalonFX(17, "canivore"); //15

  // static WPI_TalonFX motorBLDrive = new WPI_TalonFX(12, "canivore"); //15
  // static WPI_TalonFX motorBRDrive = new WPI_TalonFX(10, "canivore"); //15
  // static WPI_TalonFX motorFLDrive = new WPI_TalonFX(11, "canivore"); //15
  // static WPI_TalonFX motorFRDrive = new WPI_TalonFX(13, "canivore"); //15

  public static WPI_Pigeon2 imu = new WPI_Pigeon2(5);

  static WPI_TalonFX motorBLSteer = new WPI_TalonFX(DrivetrainConstants.LB_STEER_MOTOR_PORT); // 15
  static WPI_TalonFX motorBRSteer = new WPI_TalonFX(DrivetrainConstants.RB_STEER_MOTOR_PORT); // 15
  static WPI_TalonFX motorFLSteer = new WPI_TalonFX(DrivetrainConstants.LF_STEER_MOTOR_PORT); // 15
  static WPI_TalonFX motorFRSteer = new WPI_TalonFX(DrivetrainConstants.RF_STEER_MOTOR_PORT); // 15

  static WPI_TalonFX motorBLDrive = new WPI_TalonFX(DrivetrainConstants.LB_DRIVE_MOTOR_PORT); // 15
  static WPI_TalonFX motorBRDrive = new WPI_TalonFX(DrivetrainConstants.RB_DRIVE_MOTOR_PORT); // 15
  static WPI_TalonFX motorFLDrive = new WPI_TalonFX(DrivetrainConstants.LF_DRIVE_MOTOR_PORT); // 15
  static WPI_TalonFX motorFRDrive = new WPI_TalonFX(DrivetrainConstants.RF_DRIVE_MOTOR_PORT); // 15

  CANCoder bl_encoder = new CANCoder(DrivetrainConstants.LB_ANGLE_ENCODER_PORT);
  CANCoder fl_encoder = new CANCoder(DrivetrainConstants.LF_ANGLE_ENCODER_PORT);
  CANCoder br_encoder = new CANCoder(DrivetrainConstants.RB_ANGLE_ENCODER_PORT);
  CANCoder fr_encoder = new CANCoder(DrivetrainConstants.RF_ANGLE_ENCODER_PORT);

  Rotation2d angle = new Rotation2d(Math.toRadians(0));

  // Locations for the swerve drive modules relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(0.315, 0.315);
  Translation2d m_frontRightLocation = new Translation2d(0.315, -0.315);
  Translation2d m_backLeftLocation = new Translation2d(-0.315, 0.315);
  Translation2d m_backRightLocation = new Translation2d(-0.315, -0.315);

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  // I need to somehow define the modules first
  public SwerveDriveOdometry m_odometry;

  Pose2d m_pose = new Pose2d();

  // SwerveDriveKinematics driveKinematics;

  /** Creates a new SwerveDrive. */

  public SwerveDrive() {
    motorBLSteer.configFactoryDefault();
    motorBRSteer.configFactoryDefault();
    motorFLSteer.configFactoryDefault();
    motorFRSteer.configFactoryDefault();

    motorBLDrive.configFactoryDefault();
    motorBRDrive.configFactoryDefault();
    motorFLDrive.configFactoryDefault();
    motorFRDrive.configFactoryDefault();

    configureDriveMotor(motorBLDrive);
    configureDriveMotor(motorBRDrive);
    configureDriveMotor(motorFLDrive);
    configureDriveMotor(motorFRDrive);


    motorBLDrive.setNeutralMode(NeutralMode.Brake);
    motorFLDrive.setNeutralMode(NeutralMode.Brake);
    motorBRDrive.setNeutralMode(NeutralMode.Brake);
    motorFRDrive.setNeutralMode(NeutralMode.Brake);

    motorFLDrive.setInverted(false);  // flipped all drive
    motorFRDrive.setInverted(true);

    motorBRDrive.setInverted(false);
    motorBLDrive.setInverted(true); //true

    motorFLSteer.setInverted(true);
    motorBLSteer.setInverted(true);
    motorFRSteer.setInverted(true);
    motorBRSteer.setInverted(true);

    //imu.reset();

    double kP = DrivetrainConstants.rotational_kP; // 0.01
    double kI = DrivetrainConstants.rotational_kI;
    double kD = DrivetrainConstants.rotational_kD;

    // magnet offset (find on config page) - (absolute) current position
    br_encoder.configMagnetOffset(166.464844 - 177.715);
    bl_encoder.configMagnetOffset(139.042969  - -0.527 );  //186.240234 - 47.197
    fl_encoder.configMagnetOffset(-172.617188 - -177.275 );
    fr_encoder.configMagnetOffset(-156.181641 - -179.824 );

    // 04/03/2023
    // br_encoder.configMagnetOffset(23.5546875 - -142.998);
    // bl_encoder.configMagnetOffset(186.240234 - 47.197);  //186.240234 - 47.197
    // fl_encoder.configMagnetOffset(-312.714844 - -140.010);
    // fr_encoder.configMagnetOffset(-156.5332 - -0.264 );

    motorBLDrive.setSelectedSensorPosition(0);
    motorFLDrive.setSelectedSensorPosition(0);
    motorBRDrive.setSelectedSensorPosition(0);
    motorFRDrive.setSelectedSensorPosition(0);

    /*
     * for simba
     * //magnet offset - absolute position
     * br_encoder.configMagnetOffset(149.677734 - 0.352);
     * bl_encoder.configMagnetOffset(59.5898438 - -179.736);
     * fl_encoder.configMagnetOffset(11.953125 - 18.896);
     * fr_encoder.configMagnetOffset(104.326172 - 176.396);
     */

    configCANCoder(fl_encoder);
    configCANCoder(fr_encoder);
    configCANCoder(br_encoder);
    configCANCoder(bl_encoder);

    frontLeft = new SwerveModule(kP, kI, kD, motorFLSteer, motorFLDrive, fl_encoder);
    frontRight = new SwerveModule(kP, kI, kD, motorFRSteer, motorFRDrive, fr_encoder);
    backLeft = new SwerveModule(kP, kI, kD, motorBLSteer, motorBLDrive, bl_encoder);
    backRight = new SwerveModule(kP, kI, kD, motorBRSteer, motorBRDrive, br_encoder);

    m_odometry = new SwerveDriveOdometry(
        m_kinematics, imu.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, new Pose2d(0, 0, new Rotation2d()));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("FL drive encoder",
    // motorFLDrive.getSelectedSensorPosition());
    // SmartDashboard.putNumber("FR drive encoder",
    // motorFRDrive.getSelectedSensorPosition());
    // SmartDashboard.putNumber("BL drive encoder",
    // motorBLDrive.getSelectedSensorPosition());
    // SmartDashboard.putNumber("BR drive encoder",
    // motorBRDrive.getSelectedSensorPosition());

    // SmartDashboard.putNumber("FL drive velocity", motorFLDrive.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("FR drive velocity", motorFRDrive.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("BL drive velocity", motorBLDrive.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("BR drive velocity", motorBRDrive.getSelectedSensorVelocity());

    // SmartDashboard.putNumber("FL CANCoder", fl_encoder.getAbsolutePosition());
    // SmartDashboard.putNumber("FR CANCoder", fr_encoder.getAbsolutePosition());
    // SmartDashboard.putNumber("BL CANCoder", bl_encoder.getAbsolutePosition());
    // SmartDashboard.putNumber("BR CANCoder", br_encoder.getAbsolutePosition());

    // SmartDashboard.putNumber("FL Angle", frontLeft.getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("FR Angle", frontRight.getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("BL Angle", backLeft.getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("BR Angle", backRight.getPosition().angle.getDegrees());

    // SmartDashboard.putNumber("imu", imu.getYaw());

    // SmartDashboard.putNumber("m_pose X", m_odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("m_pose Y", m_odometry.getPoseMeters().getY());
    // SmartDashboard.putNumber("m_pose degrees", m_odometry.getPoseMeters().getRotation().getDegrees());

    // SmartDashboard.putNumber("setpoint" , frontLeft.pidDirectionController.getSetpoint());


    frontLeft.updatePID();
    frontRight.updatePID();
    backRight.updatePID();
    backLeft.updatePID();

    updateOdometry();

  }

  public void drive(double xSpeed, double ySpeed, double angularVelocity, boolean isFieldCentric) {
    SwerveDrive.isFieldCentric = isFieldCentric;
    ChassisSpeeds speeds;
    SwerveModuleState[] moduleStates;
 
    if (isFieldCentric) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, angularVelocity, m_odometry.getPoseMeters().getRotation());
    } else {
      speeds = new ChassisSpeeds(xSpeed, ySpeed, angularVelocity);
    }

    moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 3.5);

    frontLeft.setDesiredStateVelocity(moduleStates[0]);
    frontRight.setDesiredStateVelocity(moduleStates[1]);
    backLeft.setDesiredStateVelocity(moduleStates[2]);
    backRight.setDesiredStateVelocity(moduleStates[3]);

  }

  public void drivePercent(double xSpeed, double ySpeed, double angularVelocity, boolean isFieldCentric) {
    SwerveDrive.isFieldCentric = isFieldCentric;
    ChassisSpeeds speeds;
    SwerveModuleState[] moduleStates;

    if (isFieldCentric) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, angularVelocity, Rotation2d.fromDegrees(imu.getYaw()));
    } else {
      speeds = new ChassisSpeeds(xSpeed, ySpeed, angularVelocity);
    }

    // these values are also ok. but they're affecting the steering motors for some
    // reason.
    // SmartDashboard.putNumber("xSpeed", xSpeed);
    // SmartDashboard.putNumber("ySpeed", ySpeed);
    // SmartDashboard.putNumber("angularVelocity", angularVelocity);

    moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 3.5);

    // SmartDashboard.putNumber("module state", moduleStates[0].speedMetersPerSecond);

    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);

  }

  public void setSteerPosition(double steerPosition) {

    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(steerPosition)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(steerPosition)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(steerPosition)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(steerPosition)));

  }

  public void setDrivePower(double drivePower) {
    motorFLDrive.set(drivePower);
    // motorFRDrive.set(drivePower);
    // motorBRDrive.set(drivePower);
    // motorBLDrive.set(drivePower);
  }

  public void setSteerVelocity(double steerVelocity) {
    motorFLSteer.set(ControlMode.PercentOutput, steerVelocity);
    motorFRSteer.set(ControlMode.PercentOutput, steerVelocity);  
    motorBLSteer.set(ControlMode.PercentOutput, steerVelocity);
    motorBRSteer.set(ControlMode.PercentOutput, steerVelocity);

  }

  void configCANCoder(CANCoder cancoder) {
    cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
  }

  public static double deadZone(double input) {
    if (Math.abs(input) < 0.1) {
      return 0;
    } else if (input > 0.1) {
      // SmartDashboard.putNumber("Input pos deadzone", 1.25 * (input + 0.25));
      return 1.1 * input * input - 0.1 * input;//1.25 * (input - 0.2);
    } else {
      // SmartDashboard.putNumber("Input neg deadzone", 1.25 * (input - 0.25));
      return -1.1 * input * input + 0.1 * input;//1.25 * (input + 0.2);
    }
  }

  public void updateOdometry() {
    m_odometry.update(Rotation2d.fromDegrees(imu.getYaw()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
  }

  public void resetGyro(){
    imu.reset();
  }

  public void setSwervePosition(Pose2d startingPos){
    m_odometry.resetPosition(Rotation2d.fromDegrees(imu.getYaw()), 
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
      }, startingPos);
  }

  public DoubleSupplier getHeading() {
    return new DoubleSupplier() {
      @Override
      public double getAsDouble() {
        return imu.getYaw();
      }
    };
  }

  public DoubleSupplier getX() {
    return new DoubleSupplier() {
      @Override
      public double getAsDouble() {
        return m_odometry.getPoseMeters().getX();
      }
    };
  }

  public DoubleSupplier getY() {
    return new DoubleSupplier() {
      @Override
      public double getAsDouble() {
        return m_odometry.getPoseMeters().getX();
      }
    };
  }

  public static double autoXSpeed = 0;
  public static double autoYSpeed = 0;
  public static double autoAngularVelocity = 0;

  public void autoDrive() {
    double xSpeed = autoXSpeed;
    double ySpeed = autoYSpeed;
    double angularVelocity = autoAngularVelocity;

    ChassisSpeeds speeds;
    SwerveModuleState[] moduleStates;

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, angularVelocity,
        Rotation2d.fromDegrees(imu.getYaw()));

    // these values are also ok. but they're affecting the steering motors for some
    // reason.
    // SmartDashboard.putNumber("xSpeed", xSpeed);
    // SmartDashboard.putNumber("ySpeed", ySpeed);
    // SmartDashboard.putNumber("angularVelocity", angularVelocity);

    moduleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 3.5);

    // SmartDashboard.putNumber("module state", moduleStates[0].speedMetersPerSecond);

    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);

  }

  public WPI_TalonFX configureDriveMotor(WPI_TalonFX motor) {
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_5Ms;
    configuration.velocityMeasurementWindow = 8;
    configuration.slot0.kP = 0.001; // 0.06
    configuration.slot0.kD = 0; // 3
    configuration.slot0.kF = 0.046200037;

    motor.configAllSettings(configuration);
    motor.configClosedloopRamp(0.3);

    return motor;
  }

}