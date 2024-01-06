package frc.robot.subsystems;//TODO this programs the arm with working PID for mid and high positions
import frc.robot.subsystems.Wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.DemandType;


public class ArmReal extends SubsystemBase{

    double gearRatio = 161.62;
    double countsPerRev = 2048;
    double offset;

    double countsPerArmRev = gearRatio * countsPerRev;
    double kMeasuredPosHorizontal = 8.050; //Position measured when arm is horizontal
    double countsPerDegree = countsPerArmRev / 360; //Sensor is 1:1 with arm rotation
    // counts when intaking cone: 10832

    double maxPower;
    double minPower;

    public double minPos = 15;
    public double maxPos = 100000;

    private static WPI_TalonFX armMotor;
    private static Wrist m_Wrist;

    public ArmReal(){
        maxPower = 0.5;
        minPower = 0;

        // armMotor = new TalonFX(2, "CANivore");
        armMotor = new WPI_TalonFX(2, "CANivore");

        initArm();
        armMotor.setSelectedSensorPosition(0);
        
    } 


    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        //armMotor.getSelectedSensorPosition();
        printArmInfo();
        // SmartDashboard.putNumber("arm angle", countsToAngles(armMotor.getSelectedSensorPosition()));
    }

    //Question 2: Putting the motors on coast mode means they will not immediately stop and will instead spin a bit more due to inertia even though the power supply has stopped

    public double getPosition(){
        return armMotor.getSelectedSensorPosition();
    }

    public void setArm(double counts, double speed) {
        //This method was tested and the shooter did not spin when the method was called.
        // Can you see how to fix it?
        // double targetposition = angleToCounts(angle);
        armMotor.set(ControlMode.Position, counts);
        armMotor.configPeakOutputForward(speed);
        armMotor.configPeakOutputReverse(-speed);
    }

    public void setArmMotionMagic(double targetposition) {

        armMotor.configFactoryDefault();
        
        double currentPos = armMotor.getSelectedSensorPosition();
        double degrees = (currentPos - kMeasuredPosHorizontal) / countsPerDegree;//TODO measure counts at horizontal
        double radians = java.lang.Math.toRadians(degrees);
        double cosineScalar = java.lang.Math.cos(radians);

        double maxGravityFF = maxPower; // TODO Needs to be tuned
        //update from 27/2: we think the feed forward is the maxpower at horizontal, which is just the peak output

        //double targetposition = angleToCounts(angle); <-- only used when receiving angles
        armMotor.set(ControlMode.MotionMagic, targetposition, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);
    }
    
    public double angleToCounts(double angle){
        double countConversion = angle * (countsPerArmRev / 360);
        return countConversion;

    }

    public double countsToAngles(double counts){
        double angleConversion = (counts / countsPerArmRev) * 360;
        return angleConversion;
    }

    public void printArmInfo(){
        SmartDashboard.putNumber("Arm encoder counts", armMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm speed", armMotor.getMotorOutputPercent());
        // SmartDashboard.putNumber("Arm counts at 90 degrees", angleToCounts(90));
    }

    public double armDrivetrainRamping(){
        return (
            (armMotor.getSelectedSensorPosition() - minPos) / (maxPos - minPos)
            );
      }

    public void initArm(){
        armMotor.configFactoryDefault();

        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.setInverted(false);

        armMotor.config_kP(0, armWristConstants.arm_kP);
        armMotor.config_kD(0, armWristConstants.arm_kD);
        armMotor.config_kI(0, armWristConstants.arm_kI);
        armMotor.config_kF(0, armWristConstants.arm_kF);
        armMotor.configPeakOutputForward(maxPower);
        armMotor.configPeakOutputReverse(-maxPower);
    }

    // public void pickUpFloorLineup(){
    //     setArm(20, 0.6);
    //     armMotor.set(0.1);
    //     m_Wrist.setWrist(-50, 0.6);
    // }
}