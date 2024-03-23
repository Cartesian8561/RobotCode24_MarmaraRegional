// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */

  private final CANSparkMax climbMotorRight = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax climbMotorLeft = new CANSparkMax(11, MotorType.kBrushless);


  public ClimbSubsystem() {}


  public void setVolts(double volts){
    climbMotorLeft.setVoltage(volts);
    climbMotorRight.setVoltage(volts);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
