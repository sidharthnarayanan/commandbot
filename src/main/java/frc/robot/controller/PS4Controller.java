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
    public double getXSpeed() {
        return MathUtil.applyDeadband(ps4Controller.getLeftX(), OIConstants.kDriveDeadband);
    }

    @Override
    public double getYSpeed() {
        return MathUtil.applyDeadband(ps4Controller.getLeftY(), OIConstants.kDriveDeadband);
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