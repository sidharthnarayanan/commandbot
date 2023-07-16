// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final PWMSparkMax m_motor = new PWMSparkMax(IntakeConstants.kMotorPort);
  
  /** Returns a command that deploys the intake */
  public CommandBase intakeCommand() {
    return run(() -> {System.out.println("Intake in progress"); m_motor.set(1.0);}
        ).withName("Intake");
  }

  public CommandBase intakeCompleteCommand() {
    return runOnce(() -> {System.out.println("Intake complete"); m_motor.set(0.0);}
    ).withName("IntakeComplete");
  }

  /** Returns a command that turns off and retracts the intake. */
  public CommandBase releaseCommand() {
    return run(
            () -> { System.out.println("Release in progress");  m_motor.set(-1.0); }
          ).withName("Release");
  }
}
