// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Storage;

public class FeedBall extends CommandBase {
  
  public Storage storage;
  public Feeder feeder;

  public FeedBall(Storage storage, Feeder feeder) {
    this.storage = storage;
    this.feeder = feeder;
    addRequirements(feeder,storage);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    feeder.runForward();
    storage.bothForward();
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
    storage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
