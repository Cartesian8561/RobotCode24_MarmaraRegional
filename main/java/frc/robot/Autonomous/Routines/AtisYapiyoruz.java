// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous.Routines;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous.AngleCalculator;
import frc.robot.Autonomous.TeleopAutos.goToSpeaker;
import frc.robot.commands.IntakeCommands.CloseArmCommand;
import frc.robot.commands.ShooterCommands.AutoShootCommand;
import frc.robot.commands.ShooterCommands.IntakeFeedCommand;
import frc.robot.commands.ShooterCommands.OpenArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AtisYapiyoruz extends SequentialCommandGroup {
  private SwerveSubsystem swerveSubsystem;
  private ArmSubsystem armSubsystem;
  private AngleCalculator angleCalculator;
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  

  /** Creates a new AtisYapiyoruz. */
  public AtisYapiyoruz(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem, AngleCalculator angleCalculator, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.armSubsystem = armSubsystem;
    this.angleCalculator = angleCalculator;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new goToSpeaker(swerveSubsystem),
        new OpenArmCommand(armSubsystem, angleCalculator, 69, true),
        new AutoShootCommand(shooterSubsystem, angleCalculator),

        new SequentialCommandGroup(
        new WaitCommand(1.3),
        new AutoMagick(intakeSubsystem).withTimeout(0.5)
        )
      )

      );
  
  }
}
