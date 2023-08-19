package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface TeleOpController {
    //Intake Bindings
    Trigger releaseTrigger();
    Trigger coneIntakeTrigger();
    Trigger cubeIntakeTrigger();

    // Drive Bindings
    double getXSpeed();
    double getYSpeed();
    double getRotation();

    // Lift Bindings
    double getRaiseSpeed();
    double getLowerSpeed();
    Trigger raiseArmTrigger();
    Trigger lowerArmTrigger();
}
