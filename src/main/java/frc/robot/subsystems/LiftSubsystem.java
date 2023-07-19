package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem {
    private final CANSparkMax liftMotorRt = new CANSparkMax(Constants.LiftConstants.LIFT_RT, MotorType.kBrushless);
    private final CANSparkMax liftMotorLt= new CANSparkMax(Constants.LiftConstants.LIFT_LT, MotorType.kBrushless);
    private final MotorControllerGroup a_rightMotors = new MotorControllerGroup(liftMotorRt);
    private final MotorControllerGroup a_leftMotors = new MotorControllerGroup(liftMotorLt);
    private boolean stopped = true;
    private RelativeEncoder m_encoder;
    private double currSpeed = 0;
    private double stoppedPos;
    private double liftRange = 0; // Difference between high and low encode values
    private double lowLimit = 0;
    private final DifferentialDrive lift = new DifferentialDrive(a_leftMotors, a_rightMotors);

    public LiftSubsystem() {
        liftMotorRt.setIdleMode(IdleMode.kBrake);
        liftMotorLt.setIdleMode(IdleMode.kBrake);
        liftMotorRt.setInverted(false);
        liftMotorLt.setInverted(true);
        m_encoder = liftMotorRt.getEncoder();       
/*             
        m_pidController.setP(0.6);
        m_pidController.setI(0.0);
        m_pidController.setD(0.0);
*/
        stoppedPos = m_encoder.getPosition();
        SmartDashboard.putNumber(LiftConstants.LIFT_RANGE_LABEL, liftRange);
    }

    public void init() {
        lowLimit = SmartDashboard.getNumber(LiftConstants.LIFT_LOW_LIMIT, lowLimit);
        liftRange = SmartDashboard.getNumber(LiftConstants.LIFT_RANGE_LABEL, liftRange);
    }

    public void setPosition(double position) {
        m_encoder.setPosition(position);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    // Returns true if target reached
    public boolean moveToTarget(double target) {
        target += SmartDashboard.getNumber(LiftConstants.LIFT_LOW_LIMIT, 0);
        double curPos = m_encoder.getPosition();
        double diff = curPos-target;
        System.out.println("Moving lift to target:"+target+".. Diff:"+diff);
        double speed = Math.abs(diff)>10?0.75:Math.abs(diff)>2?0.5:0.25;
        if (diff>.5) { lowerArm(-speed); return false; }
        else if (diff<-0.5) { raiseArm(speed); return false; }
        else {stop(); return true;}
    }

    public void raiseArm(double speed) {
        lowLimit = SmartDashboard.getNumber(LiftConstants.LIFT_LOW_LIMIT, 0);
        double highLimit = lowLimit+liftRange;
        if (m_encoder.getPosition()>highLimit) {
            liftRange = SmartDashboard.getNumber(LiftConstants.LIFT_RANGE_LABEL, liftRange); // re=read from dashboard
            highLimit = lowLimit+liftRange;
            if (liftRange>0 && m_encoder.getPosition()>highLimit) {
                System.out.println("Can't go higher than "+highLimit);
                stop();
                return;
            }
        }
        System.out.println("raiseArm:"+speed);
        stopped = false;
        currSpeed = speed;
        lift.arcadeDrive(speed, 0);
    }

    public void lowerArm(double speed) {
        double liftRange = SmartDashboard.getNumber(LiftConstants.LIFT_RANGE_LABEL, 0);
        double lowLimit = SmartDashboard.getNumber(LiftConstants.LIFT_LOW_LIMIT, 0);
        if (liftRange>0 && m_encoder.getPosition()<lowLimit) {
            System.out.println("Cant go lower!!!");
            stop();
            return;
        }
        System.out.println("lowerArm:"+speed);
        stopped = false;
        currSpeed = speed;
        lift.arcadeDrive(speed, 0);
    }

    public boolean isStopped() {
        return stopped;
    }

    public void stop() {
        double currentPos = m_encoder.getPosition();
        if (!stopped) {
            currSpeed *= 0.5;
            stoppedPos = currentPos;
            if (currSpeed<0.25 && currSpeed>-0.25) {
                lift.stopMotor();
                stopped = true;
                System.out.println("Stopped Lift at pos:"+stoppedPos);
            } else
                System.out.println("Slowing lift motor..speed:"+currSpeed);
        }
        lift.arcadeDrive(0, 0);  
    }

    public double getCurrentSpeed() {
        return currSpeed;
    }
}