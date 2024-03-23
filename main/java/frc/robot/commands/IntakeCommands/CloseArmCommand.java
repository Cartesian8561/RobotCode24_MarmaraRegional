// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;


import org.opencv.objdetect.FaceDetectorYN;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class CloseArmCommand extends Command {

  private final ArmSubsystem faker;
  private final double m_setpoint;
  private final double m_time;
  private Timer m_timer;
  private double startTime;
  private boolean isFinished = false;

  /** Creates a new CloseArmCommand. */
  public CloseArmCommand(ArmSubsystem armSub, double setpoint, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);

    faker = armSub;
    this.m_setpoint = setpoint;
    this.m_time = time;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer = new Timer();
    m_timer.restart();
    startTime = m_timer.get();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = m_timer.get();
    //System.out.println(currentTime);
    if (currentTime - startTime < m_time) {
      faker.runTo(m_setpoint);
    }else{
      m_timer.reset();
      m_timer.stop();

      faker.setArmVoltage(0.0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    faker.setArmVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
