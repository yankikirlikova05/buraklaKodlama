// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ClimbPOV extends CommandBase {
  /** Creates a new ClimbPOV. */
  Climb climb;
  XboxController operator;
  public ClimbPOV(XboxController operator,Climb climb) {
    this.climb = climb;
    this.operator = operator;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(operator.getPOV()){
      case 0:
        climb.climbUp();
      case 45:
      climb.rightUp();
      case 90:
      //pass
      case 135:
      climb.rightDown();
      case 180:
      climb.climbDown();
      case 225:
      climb.leftDown();
      //pass
      case 315:
      climb.leftUp();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
