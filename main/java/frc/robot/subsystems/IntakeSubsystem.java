// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Helpers.ColorSensor;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  CANSparkMax intake = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax lowIntake = new CANSparkMax(5, MotorType.kBrushless);
  private ColorSensor sensor;


  public IntakeSubsystem(ColorSensor colour) {
    intake.setInverted(true);
    lowIntake.setInverted(true);
    this.sensor = colour;
  }

  

  public void setIntakeVoltage(double volts){
    intake.setVoltage(volts);
  }

  public void setLowIntakeVoltage(double volts){
    lowIntake.setVoltage(volts);
  }

  public void setBothIntake(double volts){
    setIntakeVoltage(volts);
    setLowIntakeVoltage(-volts);
  }
  public void setBothIntake(double volts1, double volts2){
    setIntakeVoltage(volts1);
    setLowIntakeVoltage(-volts2);
  }

 public void stopMotor() {
  setIntakeVoltage(0.0);
  setLowIntakeVoltage(0.0);
 }

 public void setIntakeSpeed(double speed){
  intake.set(speed);
 }

 public boolean matchColor(){
  return sensor.matchColor();
 }
 

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("sensorMETÇ", sensor.matchColor());
    SmartDashboard.putNumber("red", sensor.getDetectedColour().red);
    // This method will be called once per scheduler run
  }
}
