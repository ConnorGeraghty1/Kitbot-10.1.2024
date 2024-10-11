package frc.robot.Subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class IMU{


    
    public static ADIS16470_IMU imu = new ADIS16470_IMU();
  
  	float xOffsetIMU;// = -0.165; //IMU is 16.5cm to the left of robot's center
  	float yOffsetIMU;// = -0.254; //IMU is 25.4cm behind robot's center
    double angularVelocity;
  
  	double angularVelocity = imu.getRate(); 
  	
  	float[] vLinear = new float[3];
  	vLinear = 
  
  	robotAngularVelocity = imu.getRate() - /(Math.pow(xOffsetIMU, 2)*Math.pow(yOffsetIMU, 2)

    public IMU(){
      xOffsetIMU = -0.165; //IMU is 16.5cm to the left of robot's center
      yOffsetIMU = -0.254; //IMU is 25.4cm behind robot's center
      angularVelocity = 0;
    }
    
}
