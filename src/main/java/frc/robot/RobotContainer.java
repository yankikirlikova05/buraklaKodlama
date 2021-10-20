package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.lib.util.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.DriveForDistance;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootBallSubsystems;
import frc.robot.commands.ShooterPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Feeder;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Swerve swerveDrivetrain = new Swerve(true);
  XboxController driver = new XboxController(0);
  Joystick operator = new Joystick(1);
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  LEDSubsystem LED = new LEDSubsystem();

  
  public Shooter shooter = new Shooter();
  public ShooterPID shooterpid = new ShooterPID(shooter, 2000);
  //AutoShoot autoShoot = new AutoShoot(shooter, swerveDrivetrain);

  public Intake intake = new Intake();
  public IntakeCommand intakeCommand = new IntakeCommand(intake);
  public Storage storage = new Storage();
  public Feeder feeder = new Feeder(false);
  
  public TurnToAngle turnToAngle = new TurnToAngle(swerveDrivetrain, 45);

  public ShootBallSubsystems shootBallSubsystems = new ShootBallSubsystems(shooter, feeder, storage);

  public DriveForDistance driveForDistance = new DriveForDistance(swerveDrivetrain, 3);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton shooterButton = new JoystickButton(operator, 2);
    shooterButton.whileHeld(shooterpid);

    new JoystickButton(operator, 7).whileHeld(new RunCommand(()->feeder.runForward(), feeder));
    new JoystickButton(operator, 7).whenReleased(new RunCommand(()->feeder.stop(), feeder));
    new JoystickButton(operator, 8).whileHeld(new RunCommand(()-> storage.bothForward(), storage));
    new JoystickButton(operator, 8).whenReleased(new RunCommand(()-> storage.stop(), storage));


    JoystickButton ledButton = new JoystickButton(operator, 1);
    ledButton.whenActive(new RunCommand(()-> LED.turnOn(), LED));
    ledButton.whenReleased(new RunCommand(()-> LED.turnOff(), LED));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return intakeCommand;
  }
}
