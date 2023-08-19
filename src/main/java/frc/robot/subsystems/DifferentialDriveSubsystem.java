// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Date;
import java.util.function.DoubleSupplier;

public class DifferentialDriveSubsystem extends SubsystemBase {
  Date autonStart = null;
  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
          new PWMSparkMax(DriveConstants.kLeftMotor1Port),
          new PWMSparkMax(DriveConstants.kLeftMotor2Port));

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(
          new PWMSparkMax(DriveConstants.kRightMotor1Port),
          new PWMSparkMax(DriveConstants.kRightMotor2Port));

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  /** Creates a new Drive subsystem. */
  public DifferentialDriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> {
      if (fwd.getAsDouble()!=0 || rot.getAsDouble()!=0) 
        System.out.println("arcadeDrive:"+fwd.getAsDouble()+" with rot:"+rot.getAsDouble()); 
      m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble());}
      ).withName("arcadeDrive");
  }

  /**
   * Returns a command that drives the robot forward for a specified time at a specified speed.
   *
   * @param timeInSec The time to drive forward in seconds
   * @param speed The fraction of max speed at which to drive
   */
  public CommandBase driveTimeCommand(long timeInSec, double speed) {
    return runOnce(
            () -> {
              autonStart = new Date();
            })
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () ->  (new Date().getTime()-autonStart.getTime())/1000 >= timeInSec)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.stopMotor());
  }
}