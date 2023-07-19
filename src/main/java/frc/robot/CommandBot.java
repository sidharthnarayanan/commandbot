// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.ItemType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class CommandBot {
  // The robot's subsystems
  private final Drive m_drive = new Drive();
  private final Intake m_intake = new Intake();

  // The driver's controller
  CommandPS4Controller teleOpController =  new CommandPS4Controller(OIConstants.kDriverControllerPort);
  /**
   ****** Use this method to define bindings between conditions and commands. These are useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>Should be called during {@link Robot#robotInit()}.
   *
   * <p>Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {
    // Control the drive with split-stick arcade controls
    m_drive.setDefaultCommand( m_drive.arcadeDriveCommand(
            () -> -teleOpController.getLeftX(), () -> -teleOpController.getRightX()));
    
    // Deploy the intake with the triangle button for the cone
    teleOpController.triangle().whileTrue(m_intake.intakeCommand(ItemType.Cone));
    teleOpController.triangle().onFalse(m_intake.holdCommand());
    // Release the intake with the cross button for the cube
    teleOpController.cross().whileTrue(m_intake.releaseCommand());
    teleOpController.cross().onFalse(m_intake.stopCommand());


    // Deploy the intake with the square button for the cube
    teleOpController.square().whileTrue(m_intake.intakeCommand(ItemType.Cube));
    teleOpController.square().onFalse(m_intake.holdCommand());
    // Release the intake with the circle button for the cube
    teleOpController.circle().whileTrue(m_intake.releaseCommand());
    teleOpController.circle().onFalse(m_intake.stopCommand());
    
  }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>Scheduled during {@link Robot#autonomousInit()}.
   */

  public CommandBase getAutonomousCommand() {
    // Drive forward for 2 meters at half speed with a 3 second timeout
    return m_drive
        .driveDistanceCommand(AutoConstants.kDriveDistanceMeters, AutoConstants.kDriveSpeed)
        .withTimeout(AutoConstants.kTimeoutSeconds);
  }  
}