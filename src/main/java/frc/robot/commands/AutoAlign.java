// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  Shooter sh;
  Swerve sw;
  double kP = 0.0075;

  public AutoAlign(Shooter a, Swerve b) {
    sh = a; sw = b;    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sw, sh);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rot = Constants.Swerve.kMaxAngularSpeed * (MathUtil.clamp((sh.calculateTargetAngle() * kP),-1,1));
    sw.drive(0,0,rot, false);
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
