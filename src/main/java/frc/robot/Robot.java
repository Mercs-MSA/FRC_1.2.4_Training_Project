// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final TalonFX my_KrakenX60_Motor = new TalonFX(20);

  /* Start at position 0, use slot 0 for those settings */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Start at velocity 0, use slot 1 for those settings */
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(1);
  
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private final XboxController m_joystick = new XboxController(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // Common configuration settings that affect the behavior of the motor and control loops
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;    
    configs.CurrentLimits.StatorCurrentLimit = 120;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;    
    configs.CurrentLimits.SupplyCurrentLimit = 40;

    // Configuration settings specific to Slot 0, which is the position control loop in our setup
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second

    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0.0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    // Configuration settings specific to Slot 1, which is the velocity control loop in our setup
    configs.Slot1.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot1.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second

    configs.Slot1.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot1.kI = 0.0; // No output for integrated error
    configs.Slot1.kD = 0.0; // No output for derivative error

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = my_KrakenX60_Motor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    /* Make sure we start at position 0; this affects */
    my_KrakenX60_Motor.setPosition(0);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double desiredRotations = m_joystick.getLeftY() * 10; // Go for plus/minus 10 rotations
    if (Math.abs(desiredRotations) <= 0.1) { // Joystick deadzone
      desiredRotations = 0;
    }

    double desiredRotationsPerSecond  = m_joystick.getRightY() * 50; // Go for plus/minus 10 rotations per second
    if (Math.abs(desiredRotationsPerSecond ) <= 0.1) { // Joystick deadzone
      desiredRotationsPerSecond  = 0;
    }

    if (m_joystick.getLeftBumper()) {
      /* Use position voltage */
      my_KrakenX60_Motor.setControl(m_positionVoltage.withPosition(desiredRotations));
    }
    else if (m_joystick.getRightBumper()) {
      /* Use velocity voltage */
      my_KrakenX60_Motor.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
    }
    else {
      /* Disable the motor instead */
      my_KrakenX60_Motor.setControl(m_brake);
    }

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

}