// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.attribute.standard.Compression;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private DigitalInput breakbeam;
  private DigitalInput magLimitSwitch;
  private DoubleSolenoid solenoid;
  private Compressor compressor;
  private AnalogInput input;
  private AnalogPotentiometer potentiometer;
  private NetworkTable limelightTable;
  private Joystick joystick;
  private JoystickButton button;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    breakbeam = new DigitalInput(0);
    magLimitSwitch = new DigitalInput(1);
    compressor = new Compressor();
    solenoid = new DoubleSolenoid(1, 2);
    solenoid.set(Value.kOff);
    input = new AnalogInput(0);
    potentiometer = new AnalogPotentiometer(input);
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    limelightTable.getEntry("ledMode").setNumber(3);
    limelightTable.getEntry("pipeline").setNumber(3);
    joystick = new Joystick(0);
    button = new JoystickButton(joystick, 1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
    
    SmartDashboard.putBoolean("Breakbeam", breakbeam.get());
    SmartDashboard.putBoolean("MagSwitch", magLimitSwitch.get());
    SmartDashboard.putNumber("Potentiometer Angle", potentiometer.get() * 180.0);
    SmartDashboard.putNumber("Limelight Vertical Angle", limelightTable.getEntry("ty").getDouble(0.0));
    SmartDashboard.putNumber("Limelight Horizontal Angle", limelightTable.getEntry("tx").getDouble(0.0));
    SmartDashboard.putBoolean("Compressor Running", compressor.enabled());

    if (button.get())
      solenoid.toggle();

    if (!compressor.enabled())
      solenoid.set(Value.kForward);
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
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
