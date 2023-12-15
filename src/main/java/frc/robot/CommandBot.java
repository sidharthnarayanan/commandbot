// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.controller.AutonController;
import frc.robot.controller.PS4Controller;
import frc.robot.controller.TeleOpController;
import frc.robot.controller.XboxController;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.IntakeSubSystem.ItemType;

import java.nio.file.Path;
import java.util.Date;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class CommandBot {
  // The robot's subsystems
  private SwerveDriveSubsystem s_drive; // Swerve Drive
  private DifferentialDriveSubsystem d_drive; // Differential Drive
  private Subsystem drive;
  private IntakeSubSystem m_intake;
  private LiftSubsystem m_lift;
  TeleOpController teleOpController = OIConstants.controllerType.equals("PS4")
      ? new PS4Controller(OIConstants.kDriverControllerPort)
      : new XboxController(OIConstants.kDriverControllerPort);

  /**
   ****** Use this method to define bindings between conditions and commands. These are
   * useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>
   * Should be called during {@link Robot#robotInit()}.
   *
   * <p>
   * Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {
    System.out.println("Configring Bindings with driveType:" + DriveConstants.driveType);
    if (DriveConstants.driveType.startsWith("DIFF")) {
      d_drive = DifferentialDriveSubsystem.getInstance();
      drive = d_drive;
      // Note: Pass lamdba fn to get speed/rot and not the current speed/rot
      d_drive.setDefaultCommand(
          d_drive.driveCommand(() -> -teleOpController.getYSpeed(), () -> -teleOpController.getRotation()));
    } else {
      s_drive = SwerveDriveSubsystem.getInstance();
      drive = s_drive;
      // Control the swerve drive with split-stick controls
      s_drive.setDefaultCommand(s_drive.driveCommand(
          () -> -teleOpController.getXSpeed(),
          () -> -teleOpController.getYSpeed(),
          () -> -teleOpController.getRotation(), true, true));
      /*
       * teleOpController.moveTrigger().whileTrue(s_drive.driveCommand(() ->
       * -teleOpController.getXSpeed(),
       * () -> -teleOpController.getYSpeed(),
       * () -> -teleOpController.getRotation(), true, true));
       */
    }
    if (Constants.IntakeConstants.kMotorPort >= 0) {
      m_intake = new IntakeSubSystem();
      // Deploy the intake with the triangle button for the cone
      teleOpController.coneIntakeTrigger().whileTrue(Commands.run(() -> {m_intake.doIntake(ItemType.Cone);}));
      teleOpController.coneIntakeTrigger().onFalse(m_intake.holdCommand());
      // Release the intake with the cross button for the cube
      teleOpController.releaseTrigger().whileTrue(m_intake.releaseCommand());
      teleOpController.releaseTrigger().onFalse(m_intake.stopCommand());
      // Deploy the intake with the square button for the cube
      teleOpController.cubeIntakeTrigger().whileTrue(m_intake.intakeCommand(ItemType.Cube));
      teleOpController.cubeIntakeTrigger().onFalse(m_intake.holdCommand());
      // Release the intake with the circle button for the cube
      teleOpController.releaseTrigger().whileTrue(m_intake.releaseCommand());
      teleOpController.releaseTrigger().onFalse(m_intake.stopCommand());
    }

    if (Constants.LiftConstants.LIFT_RT >= 0) {
      m_lift = new LiftSubsystem();
      m_lift.setDefaultCommand(m_lift.arcadeDriveCommand(0));
      // Lifting the arm
      teleOpController.raiseArmTrigger().whileTrue(m_lift.raiseArmCommand(() -> teleOpController.getRaiseSpeed()));
      // Lowering the arm
      teleOpController.lowerArmTrigger().whileTrue(m_lift.lowerArmCommand(() -> -teleOpController.getLowerSpeed()));
    }
  }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>
   * Scheduled during {@link Robot#autonomousInit()}.
   */

  public Command getAutonomousCommand(Date autoStartTime) {
    return AutonController.getAutonCommand();
  }

  void periodic() {
    drive.periodic();
    GyroSubsystem.getInstance().periodic();
  }

  public Command getSwerveAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory;
    String trajectoryJSON = "paths/test.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (Exception e) {
      System.out.println(e.getMessage());
      exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          config);
    }

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        s_drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        s_drive::setModuleStates,
        s_drive);

    // Reset odometry to the starting pose of the trajectory.
    s_drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> s_drive.drive(0, 0, 0, false, false));
  }
}