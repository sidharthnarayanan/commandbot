package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    public enum ItemType {Cone, Cube};
    static final int INTAKE_CURRENT_LIMIT_A = 30; // How many amps the intake can use while picking up
    static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5; // How many amps the intake can use while holding
    static final double INTAKE_HOLD_POWER = 0.07; // Percent output for holding
    static final int RELEASE_POWER_LIMIT = 0; // Speed for releasing
    private final CANSparkMax intake;
    private ItemType currItemType;

    public Intake() {
        intake = new CANSparkMax(Constants.IntakeConstants.kMotorPort, MotorType.kBrushed);
        intake.setIdleMode(IdleMode.kCoast);
    }

     /** Returns a command that grabs the item */
  public CommandBase intakeCommand(ItemType itemType) {
    return run(() -> {
        System.out.println("Intake " + itemType + " in progress");
        double speed = itemType==ItemType.Cube ? 1.0 : -1.0;
        currItemType = itemType;
        intake.set(speed);
        intake.setSmartCurrentLimit(INTAKE_CURRENT_LIMIT_A);
    }).withName("Intake");
  }

  public CommandBase holdCommand() {
    return run(() -> {
        System.out.println(currItemType + " Holding in progress");
        double speed = currItemType==ItemType.Cube ? 0.07 : -0.07;
        intake.set(speed);
        intake.setSmartCurrentLimit(INTAKE_HOLD_CURRENT_LIMIT_A);
    }).withName("Hold");
  }

  public CommandBase releaseCommand() {
    return run(() -> {
        System.out.println(currItemType + " Releasing...");
        double speed = currItemType==ItemType.Cube ? -1.0 : 1.0;
        intake.set(speed);
        intake.setSmartCurrentLimit(RELEASE_POWER_LIMIT);
    }).withName("Release");
  }

  public CommandBase stopCommand() {
    return runOnce(() -> {
        System.out.println("Stopping...");
        currItemType = null;
        int speed = 0;
        intake.set(speed);
    }).withName("Stop");
  }
}