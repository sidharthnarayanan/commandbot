package frc.robot.utils;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDriveSubsystem;

/**
 * A command that runs the swerve drive for a specified amount of time
 *
 */
public class DiffDriveForDurationCommand extends CommandBase {
  protected Timer m_timer = new Timer();
  private final double m_duration;
  private double speed, rotation;

  /**
   * Creates a new SwerveDriveCommand. This command run the swerve drive, and end after the specified duration.
   *
   * @param seconds the time to run, in seconds
   */
  public DiffDriveForDurationCommand(int seconds, double speed, double rotation) {
    m_duration = seconds;
    SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
    this.speed = speed;
    this.rotation = rotation;
    addRequirements(DifferentialDriveSubsystem.getInstance());
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    System.out.println("Executing "+getName());
    DifferentialDriveSubsystem.getInstance().drive(speed, rotation);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
