// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.analog.adis16470.frc.ADIS16470_IMU;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  private static final double kAngleSetpoint = 0.0;
	private static final double kP = 0.005; // propotional turning constant
 
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private DifferentialDrive m_robotDrive;
  private XboxController controller;
  
  private final CANSparkMax m_leftMotor = new CANSparkMax(Constants.leftMotorID, MotorType.kBrushed);
  private final CANSparkMax m_rightMotor = new CANSparkMax(Constants.rightMotorID, MotorType.kBrushed);

  private final CANSparkMax m_leftMotor1 = new CANSparkMax(Constants.leftMotor1ID, MotorType.kBrushed);
  private final CANSparkMax m_rightMotor1 = new CANSparkMax(Constants.rightMotor1ID, MotorType.kBrushed);
	

	// IMU SETUP
	
  
	
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    imu.calibrate();
    imu.reset();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
	  
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_rightMotor.restoreFactoryDefaults();
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor1.restoreFactoryDefaults();
    m_rightMotor1.restoreFactoryDefaults();

    m_leftMotor1.follow(m_leftMotor);
    m_rightMotor1.follow(m_rightMotor, true); 

    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

      


    // m_rightMotor.setInverted(false);
    // m_leftMotor.setInverted(false);
    // m_rightMotor1.setInverted(false);
    // m_leftMotor1.setInverted(false);

    m_rightMotor.burnFlash();
    m_leftMotor.burnFlash();
    m_rightMotor1.burnFlash();
    m_leftMotor1.burnFlash();

    m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    controller = new XboxController(0);
	  
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
	  
    m_robotDrive.tankDrive(controller.getLeftY(), controller.getRightY());
    System.out.println(m_rightMotor1.getAppliedOutput());
    System.out.println(m_leftMotor1.getAppliedOutput());

    double turningValue = (kAngleSetpoint - imu.getAngle()) * kP;
		  
  }

  @Override
  public void testInit() {
	  
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
	  
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
