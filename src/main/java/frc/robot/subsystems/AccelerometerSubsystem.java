package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;

public class AccelerometerSubsystem {
    private BuiltInAccelerometer mRioAccel = new BuiltInAccelerometer();
    static private AccelerometerSubsystem self;
    double simulatedTilt;

    private AccelerometerSubsystem() {
        self = this;
    }

    public static AccelerometerSubsystem getInstance() {
        return self==null?new AccelerometerSubsystem():self;
    }

    public double getPitch() {
        return Math.atan2((-mRioAccel.getX()),
                Math.sqrt(mRioAccel.getY() * mRioAccel.getY() + mRioAccel.getZ() * mRioAccel.getZ())) * 57.3;
    }

    public double getRoll() {
        return Math.atan2(mRioAccel.getY(), mRioAccel.getZ()) * 57.3;
    }

    // returns the magnititude of the robot's tilt calculated by the root of
    // pitch^2 + roll^2, used to compensate for diagonally mounted rio
    public double getTilt() {
        if (simulatedTilt!=0) return simulatedTilt;
        double pitch = getPitch();
        double roll = getRoll();
        if ((pitch + roll) >= 0) {
            return -Math.sqrt(pitch * pitch + roll * roll);
        } else {
            return Math.sqrt(pitch * pitch + roll * roll);
        }
    }

    public void setTilt(double simulatedTilt) {
        this.simulatedTilt = simulatedTilt;
    }
}
