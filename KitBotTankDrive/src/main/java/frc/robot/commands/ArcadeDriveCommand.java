// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArcadeDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeDriveCommand extends Command {

  private ArcadeDriveSubsystem arcadeDriveSubsystem;

  private CommandXboxController driverController;

  /** Creates a new ArcadeDriveCommand. */
  public ArcadeDriveCommand(ArcadeDriveSubsystem arcadeDriveSubsystem, CommandXboxController driverController) {

    this.arcadeDriveSubsystem = arcadeDriveSubsystem;
    this.driverController = driverController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arcadeDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arcadeDriveSubsystem.arcadeDrive(driverController.getLeftY(), driverController.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
