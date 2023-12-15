package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.DiffDriveForDurationCommand;
import frc.robot.utils.SwerveDriveForDurationCommand;

public class AutonController {
    public static CommandBase getAutonCommand() {
        return DriveConstants.driveType.equals("DIFFER")? getDiffAutonomousCommand() : getSwerveAutonCommand();
    }

    private static CommandBase getSwerveAutonCommand() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(new SwerveDriveForDurationCommand(2,0.05,0,0.0));
        commandGroup.addCommands(new SwerveDriveForDurationCommand(2,-0.05,0,0.0));
        commandGroup.addCommands(new SwerveDriveForDurationCommand(2,0,0.05,0.0));
        commandGroup.addCommands(new SwerveDriveForDurationCommand(2,0,-0.05,0.0));
        commandGroup.addCommands(new SwerveDriveForDurationCommand(2,0,0,0.2));
        commandGroup.addCommands(new SwerveDriveForDurationCommand(2,0.1,0.1,0.0));

       // commandGroup.addCommands(SwerveDriveSubsystem.getInstance().driveTimeCommand(5, 0.05, 0.05, 0.0, true, true));
        return commandGroup;
    }

    private static CommandBase getDiffAutonomousCommand() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(new DiffDriveForDurationCommand(2,0.2,0.0));
        commandGroup.addCommands(DifferentialDriveSubsystem.getInstance().driveTimeCommand(3, 0.3, 0.4));
        return commandGroup;
    }
}