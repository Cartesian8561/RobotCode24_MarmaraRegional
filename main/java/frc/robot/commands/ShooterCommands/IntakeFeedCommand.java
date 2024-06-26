// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeFeedCommand extends Command {
  /** Creates a new IntakeFeedCommand. */
  private IntakeSubsystem intake;
  public IntakeFeedCommand(IntakeSubsystem susytme) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = susytme;
    addRequirements(intake);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeVoltage(7);      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeVoltage(0);
    System.out.println("intakefeed bitiş");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
