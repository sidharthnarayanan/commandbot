package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase{
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
   
    public CommandBase raiseArmCommand(DoubleSupplier speed) {
        return run(() -> {
            lift.arcadeDrive(speed.getAsDouble(), 0);
            System.out.println("Arm is lifting.... with speed: " + speed.getAsDouble());
        }); 
    }
    
    public CommandBase lowerArmCommand(DoubleSupplier speed) {
        return run(() -> {
            lift.arcadeDrive(speed.getAsDouble(), 0);
            System.out.println("Arm is lowering.... with speed: " + speed.getAsDouble());
        });
    }

    public CommandBase stopArmCommand() {
        return run(() -> {
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
        });
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