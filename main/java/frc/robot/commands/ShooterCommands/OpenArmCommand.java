// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autonomous.AngleCalculator;
import frc.robot.Constants.DahaIyiField;
import frc.robot.subsystems.ArmSubsystem;

public class OpenArmCommand extends Command {
  private ArmSubsystem armSubsystem;
  private double goal;
  private AngleCalculator calculator;
  private boolean isAuto;
  /** Creates a new ArmAngleCommand. */
  public OpenArmCommand(ArmSubsystem armSubsystem, AngleCalculator calculator, double goal, boolean isAuto) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
    this.isAuto = isAuto;
    this.goal = goal;
    this.calculator = calculator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.resetProfile();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("kedy", calculator.calculateArmAngle(DahaIyiField.speaker));
    if(isAuto){
      armSubsystem.runTo(calculator.calculateArmAngle(DahaIyiField.speaker));  
    }else{
      armSubsystem.runTo(goal);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setArmVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
