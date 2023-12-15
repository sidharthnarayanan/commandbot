package frc.robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;

public class PS4Controller implements TeleOpController {
    // The driver's controller
    CommandPS4Controller ps4Controller;

    public PS4Controller(int port) {
        ps4Controller = new CommandPS4Controller(port);
    }

    @Override
    public Trigger releaseTrigger() {
        return ps4Controller.cross();
    }

    @Override
    public Trigger coneIntakeTrigger() {
        return ps4Controller.triangle();
    }

    @Override
    public Trigger cubeIntakeTrigger() {
        return ps4Controller.square();
    }

    @Override
    public Trigger moveTrigger() {
        return ps4Controller.circle();
    }

    @Override
    public double getXSpeed() {
        double leftx = ps4Controller.getLeftX();
        if (leftx>0.2)
            leftx=0.2;
        else if (leftx<-0.2)
            leftx=-0.2;
        //System.out.println("xspeed:"+leftx);
        return MathUtil.applyDeadband(leftx, OIConstants.kDriveDeadband);
    }

    @Override
    public double getYSpeed() {
        double lefty = ps4Controller.getLeftY();
        if (lefty>0.2)
            lefty=0.2;
        else if (lefty<-0.2)
            lefty=-0.2;
        return MathUtil.applyDeadband(lefty, OIConstants.kDriveDeadband);
    }

    @Override
    public double getRotation() {
        return MathUtil.applyDeadband(ps4Controller.getRightX(), OIConstants.kDriveDeadband);
    }

    @Override
    public Trigger raiseArmTrigger() {
        return ps4Controller.L2();
    }

    @Override
    public Trigger lowerArmTrigger() {
        return ps4Controller.R2();
    }

    @Override
    public double getRaiseSpeed() {
        return ps4Controller.getL2Axis();
    }

    @Override
    public double getLowerSpeed() {
        return ps4Controller.getR2Axis();
    }
}