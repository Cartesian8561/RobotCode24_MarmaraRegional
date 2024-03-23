// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.io.Console;
import java.lang.invoke.ConstantBootstraps;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
  /** Creates a new ArmSubsystem. */
  private CANSparkMax armMotor = new CANSparkMax(27, MotorType.kBrushless);
  private RelativeEncoder encoder = armMotor.getEncoder();

  private final PIDController m_arm_pid = new PIDController(2.4, 0, 0.0); //kd yoktu
  private final double voltConstant = 1.3;
  //private DigitalOutput limitSwitch = new DigitalOutput(0);
  //private double offSet = encoder.getPosition();
  //private boolean anan = false;
  private ArmFeedforward feedforward = new ArmFeedforward(0.0, 1.3, 0.68);//0.78
  public PIDController pid = new PIDController(2.8, 0.0, 0.0);//20.4
  private static double kDt = 0.02;
  private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.75,1.75));
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();


  public ArmSubsystem() {
    
    armMotor.setInverted(false);
    encoder.setPosition(0.0);
    /* 
    var mm_config = new MotionMagicConfigs();

    armMotor.getConfigurator().refresh(mm_config); 
    mm_config.withMotionMagicAcceleration(0.05);
    mm_config.MotionMagicAcceleration = 0.05;
    mm_config.MotionMagicCruiseVelocity = 0.2;
    armMotor.getConfigurator().apply(mm_config);
    */
  }
  
  public State useState(double setPoint){

    goal = new TrapezoidProfile.State(setPoint, 0);

    setpoint = profile.calculate(kDt ,setpoint, goal);
    SmartDashboard.putNumber("setPoint velocity", setpoint.velocity);
    SmartDashboard.putNumber("setPoint Positoin", setpoint.position);

    return setpoint;
  }

  public void runTo(double goal){
    double gain = MathUtil.clamp(feedforward.calculate(getArmAngle() ,useState(goal).velocity) + pid.calculate(getArmAngle(), useState(goal).position), -7.0, 7.0);
    SmartDashboard.putNumber("gain", gain);
    setArmVoltage(gain);
  }

  public void resetProfile(){
    setpoint = new TrapezoidProfile.State();
  }
  

  private double getHolderVolts(){
    double volts = voltConstant * Math.cos(getArmAngle() + Math.toRadians(30));
    
    return volts;
  }

  public void stop(){
    armMotor.stopMotor();
  }



  public void moveArmPid(double setpoint){
    double volt = MathUtil.clamp(m_arm_pid.calculate(getArmAngle(), setpoint)+ getHolderVolts(), -10.0, 10.0);
    SmartDashboard.putNumber("annen", volt);
    setArmVoltage(volt);
  }


  public void setFalconSpeed(double speed){
    armMotor.set(speed);
  }

  public void setSpeed(double speed){
    armMotor.set(speed);
  }

  public void setArmVoltage(double volts){
    armMotor.setVoltage(volts);
  }


 

  public double getArmAngle(){
    //return ((encoder.getPosition() - offSet) / 5.0 * Math.PI / 2 + 0.37);
    return ((encoder.getPosition()/3.92 * 1.09) + 0.476);//0.476
    //3.92 Math.pI/2
    //1.09
  }

  public void holdArm(){
    setArmVoltage(feedforward.calculate(getArmAngle(), 0.0));
  }


  @Override
  public void periodic() {
    /*
  if(!limitSwitch.get() && anan){
      offSet = encoder.getPosition();
    }
    anan = limitSwitch.get();
    SmartDashboard.putBoolean("limit switch", !limitSwitch.get());
    */


    SmartDashboard.putNumber("arm encoder", getArmAngle());
    SmartDashboard.putNumber("falcon", encoder.getPosition());
    //SmartDashboard.putNumber("offset", offSet);
    SmartDashboard.putNumber("arm voltage", feedforward.calculate(getArmAngle(), 0.0));
  }


}