package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController implements TeleOpController{
    CommandXboxController xboxController;

    public XboxController(int port) {
        xboxController = new CommandXboxController(port);
    }

    @Override
    public Trigger moveTrigger() {
        return xboxController.leftTrigger();
    }

    @Override
    public Trigger releaseTrigger() {
        return xboxController.a();
    }

    @Override
    public Trigger coneIntakeTrigger() {
        return xboxController.b();
    }

    @Override
    public Trigger cubeIntakeTrigger() {
        return xboxController.x();
    }

    @Override
    public double getXSpeed() {
        return xboxController.getLeftX();
    }

    @Override
    public double getYSpeed() {
        return xboxController.getLeftX();
    }

    @Override
    public Trigger raiseArmTrigger() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Trigger lowerArmTrigger() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getRaiseSpeed() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getLowerSpeed() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRotation() {
        // TODO Auto-generated method stub
        return 0;
    }

}


