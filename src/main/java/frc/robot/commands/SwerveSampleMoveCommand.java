package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.DriveConstants;
import java.util.function.Supplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveSampleMoveCommand extends CommandBase {

private boolean finished;
  private boolean xflag;
  private boolean yflag;
  SwerveDriveSubsystem swerveSubsystem;
  Supplier<Boolean> switchOverride;
  Supplier<Double> headingFunction, setpointFunction;

  private PIDController vXController, vYController;
  private PIDController thetaController;

  private double vX;
  private double vY;

  private double xSetpoint;
  private double ySetpoint;
  private double tSetpoint;

  private boolean reset;
  private double tolerance;

  private double maxdrivespeed;
  private double maxturnspeed;


  private Pose2d resetPose;

  // Command constructor
  // x/y setpoint is in meters, forward is +ve for x and left is +ve for y
  // tSetpoint is in radians and posive for counter clockwise
  // If you want to reset pose then set reset to true and pass pose for resetPose
  // tolerance tells how much accurate the tolerance to distance.
  public SwerveSampleMoveCommand(SwerveDriveSubsystem swerveSubsystem,
         double xSetpoint, double ySetpoint, double tSetpoint, boolean reset, Pose2d resetPose,
         double tolerance)
  {

     this.reset = reset;
     this.resetPose = resetPose;

    addRequirements(swerveSubsystem);

    this.tolerance = tolerance;


    this.swerveSubsystem = swerveSubsystem;

    this.tSetpoint = tSetpoint;
    this.xSetpoint = xSetpoint;
    this.ySetpoint = ySetpoint;

    
    vX = 0;
    vY = 0;

    finished = false;

    maxdrivespeed = DriveConstants.kMaxSpeedMetersPerSecond;
    maxturnspeed = DriveConstants.kMaxAngularSpeed;

  }

  public void setMaxSpeeds(double drivespeed, double turnspeed) {
    maxdrivespeed = drivespeed;
    maxturnspeed = turnspeed;
  }

  @Override
  public void initialize() {
  
    if(reset){ swerveSubsystem.resetOdometry(resetPose);}

    finished = false;
    xflag = false;
    yflag = false;

    //vXController = new PIDController(6, 0.006, 0.008);
    //vYController = new PIDController(6, 0.006, 0.008);

    //change value of kp to control max speed
    vXController = new PIDController(maxdrivespeed, 0.006, 0.008);
    vYController = new PIDController(maxdrivespeed, 0.006, 0.008);


    thetaController = new PIDController(0.1, 0.004, 0.02);
    thetaController.enableContinuousInput(0, 360);

  
  }

  boolean didReachXYSetPoint() {
    double currentPoseY = swerveSubsystem.getPose().getY();
    double currentPoseX = swerveSubsystem.getPose().getX();

    SmartDashboard.putNumber("YDIFF",Math.abs(currentPoseY-ySetpoint));
    SmartDashboard.putNumber("XDIFF",Math.abs(currentPoseX-xSetpoint));

    if (Math.abs(currentPoseY-ySetpoint) <= tolerance && 
        Math.abs(currentPoseX - xSetpoint) <= tolerance
      ) {
        return true;
      }
    return false;
  }

  @Override
  public void execute(){
      /*if(swerveSubsystem.getPose().getY()  > (ySetpoint) - tolerance){
        if(swerveSubsystem.getPose().getY()  < (ySetpoint) + tolerance){
            yflag = true;
          }
      }

      if(swerveSubsystem.getPose().getX()  > (xSetpoint) - tolerance){
        if(swerveSubsystem.getPose().getX()  < (xSetpoint) + tolerance){
            xflag = true;
          }
      }

      if(yflag && xflag){ finished = true;}*/

      if (didReachXYSetPoint()) {
        finished = true;
      }

      double turningSpeed;

      turningSpeed = -thetaController.calculate(swerveSubsystem.getHeading(), tSetpoint);

      turningSpeed = (turningSpeed > maxturnspeed)?maxturnspeed:turningSpeed;

      turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;

      SmartDashboard.putNumber("ROT CACL", turningSpeed);
      SmartDashboard.putNumber("ODO Y", swerveSubsystem.getPose().getY());
      SmartDashboard.putNumber("ODO X", swerveSubsystem.getPose().getX());
      SmartDashboard.putNumber("ROBO DEG", swerveSubsystem.getHeading());
      SmartDashboard.putBoolean("ISFinished", finished);

  

      vX = vXController.calculate(swerveSubsystem.getPose().getX(), xSetpoint); // X-Axis PID
      vY = vYController.calculate(swerveSubsystem.getPose().getY(), ySetpoint); // Y-Axis PID

      SmartDashboard.putNumber("vX", vX);
      SmartDashboard.putNumber("vY", vY);

      // Create chassis speeds  
      /* 
     ChassisSpeeds chassisSpeeds;
  
      // Apply chassis speeds with desired velocities
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, turningSpeed, swerveSubsystem.getRotation2d());

      // Create states array
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
      // Move swerve modules
      swerveSubsystem.setModuleStates(moduleStates);
      */
      swerveSubsystem.drive(vX, vY, turningSpeed, true, true);

  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted){swerveSubsystem.stopModules();}

  @Override
  public boolean isFinished(){return finished;}
  
}
