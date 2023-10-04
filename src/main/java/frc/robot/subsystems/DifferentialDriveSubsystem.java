// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Date;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DifferentialDriveSubsystem extends SubsystemBase {
  Date autonStart = null;

  private final CANSparkMax m_rightDrive1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless); 
  private final CANSparkMax m_rightDrive2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);
  private final CANSparkMax m_leftDrive1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_leftDrive2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);

  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftDrive1, m_leftDrive2);

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightDrive1, m_rightDrive2);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  RelativeEncoder rtEncoder1 = m_rightDrive1.getEncoder();

  static DifferentialDriveSubsystem self;

  /** Creates a new Drive subsystem. */
  private DifferentialDriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);
  }

  static public DifferentialDriveSubsystem getInstance() {
    if (self==null) self = new DifferentialDriveSubsystem();
    return self;
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public CommandBase driveCommand(DoubleSupplier speed, DoubleSupplier rotation) {
    return driveCommand(speed.getAsDouble(), rotation.getAsDouble());
  }

  public CommandBase driveCommand(Double speed, Double rotation) {
    return run(() -> { this.drive(speed, rotation); }).withName("diffDrive");
  }

  public void drive(double speed, double rotation) {
    if (speed!=0 || rotation!=0) 
        System.out.println("arcadeDrive:"+speed+" with rot:"+rotation); 
    // A split-stick arcade command
    m_drive.arcadeDrive(speed, rotation);
  }

  /**
   * Returns a command that drives the robot forward for a specified time at a specified speed.
   *
   * @param timeInSec The time to drive forward in seconds
   * @param speed The fraction of max speed at which to drive
   */
  public CommandBase driveTimeCommand(long timeInSec, double speed, double rotation) {
    return driveCommand(speed, rotation).withTimeout(timeInSec);
  }
}