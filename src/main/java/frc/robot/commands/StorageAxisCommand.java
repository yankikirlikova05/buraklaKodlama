// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

public class StorageAxisCommand extends CommandBase {
  /** Creates a new StorageAxisCommand. */
  Storage storage;
  XboxController operator;
  public StorageAxisCommand(XboxController operator, Storage storage) {
    this.storage = storage;
    this.operator = operator;
    addRequirements(storage);
    // Use addRequirements(add) here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(operator.getX(Hand.kLeft)> 0.2){
      storage.rightForward();
    }
    else if(operator.getX(Hand.kLeft)< -0.2){
      storage.leftForward();
    }

    /*if(operator.getY(Hand.kLeft)>0.2){
      storage.bothBackward();
    }
*/
    if(Math.abs(operator.getTriggerAxis(Hand.kRight))> 0.2){
      storage.bothBackward();
    }
    else storage.stop();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    storage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
