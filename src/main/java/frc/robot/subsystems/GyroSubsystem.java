package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroSubsystem {
    //private ADXRS450_Gyro gyro1 = new ADXRS450_Gyro();
    final AHRS ahrsGyro = new AHRS(SerialPort.Port.kUSB);
    double simulatedYaw = 0, simulatedPitch=0;
    static GyroSubsystem self; // singleton
    private AccelerometerSubsystem acc = AccelerometerSubsystem.getInstance();

    public static final String GYRO_PITCH="Gyro Pitch";
    public static final String GYRO_ROLL="Gyro Roll";
    public static final String GYRO_YAW="Gyro Yaw";
    public static final String GYRO_ANGLE = "Gyro Angle";

    private GyroSubsystem() {
        init();
        self = this;
    }

    public static GyroSubsystem getInstance() {
        return self==null?new GyroSubsystem():self;
    }

    public void init() {
        if (ahrsGyro!=null) ahrsGyro.reset();
        simulatedYaw = 0;
    }

    public double getRate() {
        return ahrsGyro==null ? 0 : ahrsGyro.getRate();
    }

    public double getAngle() {
        return ahrsGyro==null ? 0 : ahrsGyro.getAngle();
    }

    public double getYaw() {
        return simulatedYaw!=0 ? simulatedYaw: ahrsGyro==null ? 0 : ahrsGyro.getYaw();
    }

    public double getRoll() {
        return ahrsGyro==null ? 0 : ahrsGyro.getRoll();
    }

    public double getPitch() {
        return simulatedPitch!=0 ? simulatedPitch : ahrsGyro==null ? 0 : ahrsGyro.getPitch();// - acc.getTilt();
    }

    public void simulationPeriodic(Double yawChange, Double pitchChange) {
        if (pitchChange!=null) {
            simulatedPitch += pitchChange;
            if (simulatedPitch<0) simulatedPitch=0;
            else if (simulatedPitch>90) simulatedPitch=90;
        }
        if (yawChange!=null) {
            simulatedYaw += yawChange;
            if (simulatedYaw>180) simulatedYaw=-180+(simulatedYaw-180);
            if (simulatedYaw<=-180) simulatedYaw=180+(simulatedYaw+180);
        }
    }

    public Rotation2d getRotation2d() {
        return ahrsGyro==null ? new Rotation2d() : ahrsGyro.getRotation2d();
    }

    public void reset() {
        if (ahrsGyro!=null) ahrsGyro.reset();
        simulatedYaw = 0;
        simulatedPitch = 0;
    }

    public void periodic() {
        //System.out.println("gyroP:"+gyro.getPitch()+", gyroR:"+gyro.getDegrees());
        //if ((tickCount & 0x1111) == 0x1111) 
        {
            SmartDashboard.putNumber(GYRO_PITCH, getPitch());
            SmartDashboard.putNumber(GYRO_YAW, getYaw());
        }
        //SmartDashboard.putNumber(DriveController.GYRO_ANGLE, gyro.getAngle());
        //SmartDashboard.putNumber(DriveController.GYRO_ROLL, gyro.getRoll());
    }
}