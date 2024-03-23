// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.List;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.AngleCalculator;
import frc.robot.Autonomous.Routines.AtisYapiyoruz;
import frc.robot.Autonomous.TeleopAutos.goToSpeaker;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DahaIyiField;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Helpers.ColorSensor;
import frc.robot.commands.IntakeCommands.CloseArmCommand;
import frc.robot.commands.IntakeCommands.grabNote;
import frc.robot.commands.ShooterCommands.AutoShootCommand;
import frc.robot.commands.ShooterCommands.OpenArmCommand;
import frc.robot.commands.SwerveCommands.DriveToPoseCommand;
import frc.robot.commands.SwerveCommands.SwerveJoystickCmd;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {

  private final CameraSubsystem cameraSubsystem = new CameraSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(cameraSubsystem);
  private final ColorSensor sensor = new ColorSensor();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(null);
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final ArmSubsystem faker = new ArmSubsystem();
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick annenJoystick = new Joystick(1);
  private final AngleCalculator calculator = new AngleCalculator(swerveSubsystem);
  private final IntakeSubsystem intake = new IntakeSubsystem(sensor);
  private final SendableChooser<Command> m_chooser;
  private CloseArmCommand m_CloseArmCommand;

  public RobotContainer() {

    NamedCommands.registerCommand("runto", new RunCommand(()-> faker.runTo(1.6)));
    NamedCommands.registerCommand("autoshoot", new AutoShootCommand(shooterSubsystem, calculator));
    NamedCommands.registerCommand("openArm", new OpenArmCommand(faker, calculator, 0, true));
    NamedCommands.registerCommand("allign", new goToSpeaker(swerveSubsystem));
    NamedCommands.registerCommand("intake", new RunCommand(() -> intake.setBothIntake(-4.0, -8.0), intake));
    NamedCommands.registerCommand("closeArm", new CloseArmCommand(faker, 0.53, 1.0));
    NamedCommands.registerCommand("SHOOT", new AtisYapiyoruz(swerveSubsystem, faker, calculator, shooterSubsystem, intake));
    //NamedCommands.registerCommand("grabpnote", new grabNote(intake, shooterSubsystem));

    m_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(m_chooser);


    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
    () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis), //buna bak
    () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
    () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
    //() -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
    () -> driverJoystick.getRawAxis(3),
    () -> driverJoystick.getRawButton(OIConstants.kDriverNoteButtonIdx)));
    configureBindings();




  }

  private void configureBindings() {

    new JoystickButton(driverJoystick, 1).onTrue(new InstantCommand(() -> {
      Constants.DahaIyiField.fieldFront = new Rotation2d(
      swerveSubsystem.getRotation2d().getRadians() + DahaIyiField.fieldFront.getRadians());
      swerveSubsystem.resetOrientation();}));
    //new JoystickButton(driverJoystick, 2).whileTrue(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.2, 4.3, 30).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.6, 5.1, 0)).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 13.8, 5.1, 0)).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.6, 5.1, 0)).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 13.8, 6.9, -30)).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.6, 5.1, 0))).onFalse(new InstantCommand(() -> swerveSubsystem.stopModules()));
    
    //new JoystickButton(driverJoystick, 2).whileTrue(new RunCommand(() -> shooterSubsystem.setShooterSpeed(0.8), shooterSubsystem)).onFalse(new InstantCommand(() -> shooterSubsystem.setShooterVoltage(0.0), shooterSubsystem));
    //new JoystickButton(driverJoystick, 3).whileTrue(new RunCommand(() -> shooterSubsystem.setShooterSpeed(1.0), shooterSubsystem)).onFalse(new InstantCommand(() -> shooterSubsystem.setShooterSpeed(0.0), shooterSubsystem));
    //new JoystickButton(driverJoystick, 7).onTrue(new grabNote(intake, shooterSubsystem));//1.9
    new JoystickButton(annenJoystick, 4).whileTrue(new OpenArmCommand(faker, calculator, (0.55), true)).onFalse(new CloseArmCommand(faker, 0.53, 1.0)); //AMP => 1.8
    new JoystickButton(annenJoystick, 2).whileTrue(new OpenArmCommand(faker, calculator, (0.95), false)).onFalse(new CloseArmCommand(faker, 0.53, 1.0)); //AMP => 1.8
//0.95 source
    new JoystickButton(annenJoystick, 8).whileTrue(new RunCommand(() -> shooterSubsystem.setShooterVoltage(3.0), shooterSubsystem)).onFalse(new InstantCommand(() -> shooterSubsystem.setShooterVoltage(0.0), shooterSubsystem));

    new JoystickButton(driverJoystick, 5).onTrue(new RunCommand(() -> intake.setBothIntake(4.0, 8.0), intake)).onFalse(new InstantCommand(() -> intake.setBothIntake(0.0), intake));
    new JoystickButton(driverJoystick, 6).onTrue(new RunCommand(() -> intake.setBothIntake(-4.0, -8.0), intake)).onFalse(new InstantCommand(() -> intake.setBothIntake(0.0), intake));
    new POVButton(driverJoystick, 90).whileTrue(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, Constants.DahaIyiField.amp.getX(), Constants.DahaIyiField.amp.getY(), Constants.DahaIyiField.amp.getRotation().getDegrees())).onFalse(new InstantCommand(()-> swerveSubsystem.stopModules()));
    new POVButton(driverJoystick, 270).whileTrue(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, Constants.DahaIyiField.source.getX(), Constants.DahaIyiField.source.getY(), Constants.DahaIyiField.source.getRotation().getDegrees())).onFalse(new InstantCommand(()-> swerveSubsystem.stopModules()));

    //new JoystickButton(driverJoystick, 6).whileTrue(new RunCommand(() -> climbSubsystem.runClimbMotorVolts(10))).onFalse(new InstantCommand(() -> climbSubsystem.runClimbMotorVolts(0)));
    //new JoystickButton(driverJoystick, 2).whileTrue(new OpenArmCommand(faker, 0.45)).onFalse(new InstantCommand(() -> faker.stop(), faker));
    //new JoystickButton(driverJoystick, 4).whileTrue(new RunCommand(() -> faker.holdArm(), faker)).onFalse(new InstantCommand(() -> faker.stop(), faker));
    //new JoystickButton(driverJoystick, 5).whileTrue(new RunCommand(() -> faker.setSpeed(0.3), faker)).onFalse(new InstantCommand(() -> faker.stop()));

    new JoystickButton(driverJoystick, 3).whileTrue(new RunCommand(() -> intake.setIntakeVoltage(-4.0), intake)).onFalse(new InstantCommand(() -> intake.setIntakeVoltage(0.0), intake));
    
    //new JoystickButton(driverJoystick, 2).whileTrue(autoRoutine1).onFalse(new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem));

    new Trigger(() -> annenJoystick.getRawAxis(3) > 0.5).whileTrue(new AutoShootCommand(shooterSubsystem, calculator)).onFalse(new InstantCommand(() -> shooterSubsystem.setShooterVoltage(0.0)));
    new Trigger(() -> annenJoystick.getRawAxis(2) > 0.5).whileTrue(new RunCommand(() -> shooterSubsystem.setShooterVoltage(-2.0), shooterSubsystem)).onFalse(new InstantCommand(() -> shooterSubsystem.setShooterSpeed(0.0)));
    new POVButton(annenJoystick, 0).whileTrue(new RunCommand(() -> climbSubsystem.setVolts(10), climbSubsystem)).onFalse(new InstantCommand(() -> climbSubsystem.setVolts(0), climbSubsystem));
    new POVButton(annenJoystick, 180).whileTrue(new RunCommand(() -> climbSubsystem.setVolts(-10), climbSubsystem)).onFalse(new InstantCommand(() -> climbSubsystem.setVolts(0), climbSubsystem));


    
    //new JoystickButton(emergencyJoystick, 8).whileTrue(new RunCommand(() -> climbSubsystem.moveServo(1), climbSubsystem, climbSubsystem)).onFalse(new InstantCommand(() -> climbSubsystem.moveServo(0), climbSubsystem));
  }

  public Command getAutonomousCommand() {
/* 
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);
    
     
    Trajectory trajectoryone = new Trajectory();
    String trajectoryJSON = new String("output/output/Unnamed_0.wpilib.json");
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectoryone = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }
*/
    
    //Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(), new Pose2d(3.0,0.0, new Rotation2d(0)), config);
    // Trajectory trajectory =  TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)), List.of(), new Pose2d(-1,0, new Rotation2d(0)), config);

    /* 
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectoryone, swerveSubsystem::getEstimatedPose, DriveConstants.kDriveKinematics, xController, yController, thetaController,swerveSubsystem::desiredRot
    ,swerveSubsystem::setModuleStates, swerveSubsystem);

*/
    //swerveSubsystem.resetOdometry(trajectory.getInitialPose());
    
    //return new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.2, 5.5, 0).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.2, 0, 0)).andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.3, 6.2, 94));
    //return new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 13.0, 5.5, 0).andThen(swerveControllerCommand.andThen(swerveControllerCommandtwo).andThen(swerveControllerCommandthree).andThen(swerveControllerCommandfour).andThen(new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem)));
    //return new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.2, 5.5, 0).andThen(new InstantCommand(() -> swerveSubsystem.stopModules()));
    return (m_chooser.getSelected());
    //return besdakka.andThen(new DriveToPoseCommand(cameraSubsystem, swerveSubsystem, 14.2, 6.5, 0));
    //return null;
  }

}
