// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.Timer;// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto3Ball extends SequentialCommandGroup {
  /** Creates a new Auto3Ball. */
  public Auto3Ball(
    Shooter shooter,
    Feeder feeder,
    Storage storage,
    Swerve swerve,
    Vision vis, 
    LEDSubsystem led){


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
      new AutoAlign(vis, swerve, led),
      new RunCommand(()-> shooter.setShooter(1.0), shooter).withTimeout(4),
      new ParallelCommandGroup(
        new RunCommand(()-> shooter.setShooter(1.0), shooter),
        new RunCommand(() -> feeder.runForward(), feeder),
        new RunCommand(() -> storage.bothForward(), storage)
      ).withTimeout(6),
      new RunCommand(()-> swerve.drive(0, 0.4, 0, true), swerve){
        @Override
        public void end(boolean interrupted){
          swerve.drive(0, 0, 0, true);
        }}.withTimeout(2)
      );

  }
}