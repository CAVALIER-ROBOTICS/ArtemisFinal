// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import javax.naming.PartialResultException;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbCommands.HomeLeftClimbCommand;
import frc.robot.commands.ClimbCommands.HomeRightClimbCommand;
import frc.robot.commands.DriveCommands.FieldDriveCommand;
import frc.robot.commands.DriveCommands.RobotDriveCommand;
import frc.robot.commands.ShootCommands.HomeHoodCommand;
import frc.robot.commands.ShootCommands.HoodCommand;
import frc.robot.commands.ShootCommands.ShootCommand;
import frc.robot.commands.TurretCommands.AimCommand;
import frc.robot.commands.TurretCommands.StartTurretCommand;
import frc.robot.commands.TurretCommands.TurnTurretCommand;
import frc.robot.subsystems.RightClimbSubsystem;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperFloorSubsystem;
import frc.robot.subsystems.HopperWallSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LeftClimbSubsytem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);


  DriveTrainSubsystems driveSub = new DriveTrainSubsystems();
  TurretSubsystem turretSub = new TurretSubsystem();;
  HopperFloorSubsystem floorSub = new HopperFloorSubsystem();
  HopperWallSubsystem wallSub = new HopperWallSubsystem();
  IntakeSubsystem intakeSub = new IntakeSubsystem();
  ShooterSubsystem shooterSub = new ShooterSubsystem();
  HoodSubsystem hoodSub = new HoodSubsystem();
  KickerSubsystem kickSub = new KickerSubsystem();
  RightClimbSubsystem rightClimb = new RightClimbSubsystem();
  LeftClimbSubsytem leftClimb = new LeftClimbSubsytem();

  PathPlannerTrajectory path;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // SmartDashboard.putNumber("Hood Angle input", 14);
    // SmartDashboard.putNumber("RPM input", 1800);
    path = PathPlanner.loadPath("ComplexAuto", 2, 1);

    // Configure the button bindings
    configureButtonBindings();

    

    turretSub.setDefaultCommand(new SequentialCommandGroup(
      new TurnTurretCommand(turretSub),
      new StartTurretCommand(turretSub),
      new AimCommand(turretSub)));

    // hoodSub.setDefaultCommand(
    //   // new HomeHoodCommand(hoodSub),
    //   new HoodCommand(hoodSub));

    rightClimb.setDefaultCommand(new RunCommand(() -> rightClimb.setClimb(()-> operator.getRightY()), rightClimb));
    leftClimb.setDefaultCommand(new RunCommand(() -> leftClimb.setClimb(()-> operator.getLeftY()), leftClimb));



    //passes conditional command into the default command of drive
    driveSub.setDefaultCommand(
      new FieldDriveCommand(
        () -> modifyAxis(driver.getLeftY()) * DriveTrainSubsystems.maxVelocityPerSecond,
        () -> modifyAxis(driver.getLeftX()) * DriveTrainSubsystems.maxVelocityPerSecond,
        () -> modifyAxis(driver.getRightX()) * DriveTrainSubsystems.maxAngularVelocityPerSecond,
        driveSub
      ));
        
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

    
    JoystickButton intake = new JoystickButton(driver, 5);
    JoystickButton outake = new JoystickButton(driver, 6);
    JoystickButton reset = new JoystickButton(driver, 4);
    JoystickButton changeDrive = new JoystickButton(driver, 3);
    JoystickButton shoot = new JoystickButton(operator, 5);
    JoystickButton kicker = new JoystickButton(operator, 6);
    JoystickButton climbUp = new JoystickButton(operator, 4);
    JoystickButton climbDown = new JoystickButton(operator, 1);
    JoystickButton homeClimb = new JoystickButton(operator, 2);


              
    intake.whileActiveContinuous(
      new ParallelCommandGroup(
        new StartEndCommand(
          ()-> intakeSub.setIntakeMotor(1), 
          ()-> intakeSub.setIntakeMotor(0),
          intakeSub),
        new StartEndCommand(
          ()-> wallSub.setWall(.3), 
          ()-> wallSub.setWall(0),
          wallSub),
          new StartEndCommand(
          () -> floorSub.setFloor(.5), 
          () -> floorSub.setFloor(0),
          floorSub))
      );

    outake.whileActiveContinuous(
      new ParallelCommandGroup(
        new StartEndCommand(
          ()-> intakeSub.setIntakeMotor(-1), 
          ()-> intakeSub.setIntakeMotor(0),
          intakeSub),
        new StartEndCommand(
          ()-> wallSub.setWall(-.3), 
          ()-> wallSub.setWall(0),
          wallSub),
        new StartEndCommand(
          ()-> floorSub.setFloor(-.3),
          ()-> floorSub.setFloor(0), 
          floorSub)
      ));
    
    homeClimb.whenPressed(new ParallelCommandGroup(
      new HomeLeftClimbCommand(leftClimb),
      new HomeRightClimbCommand(rightClimb)
    ));

    // climbUp.whileActiveContinuous(new ParallelCommandGroup(
    //   new StartEndCommand(
    //     () -> rightClimb.setClimb(0.8),
    //     () -> rightClimb.setClimb(0), 
    //     rightClimb),
    //   new StartEndCommand(
    //     () -> leftClimb.setClimb(0.8),
    //     () -> leftClimb.setClimb(0.0),
    //     leftClimb)));

    // climbDown.whileActiveContinuous(new ParallelCommandGroup(
    //   new StartEndCommand(
    //     () -> rightClimb.setClimb(-0.8),
    //     () -> rightClimb.setClimb(0), 
    //     rightClimb),
    //   new StartEndCommand(
    //     () -> leftClimb.setClimb(-0.8),
    //     () -> leftClimb.setClimb(0),
    //     leftClimb)
    // ));

  

    shoot.whileActiveContinuous(new ShootCommand(shooterSub));

      kicker.whileActiveContinuous(new ParallelCommandGroup(
        new StartEndCommand(
          ()-> kickSub.setKicker(-.8), 
          ()-> kickSub.setKicker(0.0),
          kickSub),
          new StartEndCommand(
          () -> floorSub.setFloor(.5), 
          () -> floorSub.setFloor(0),
          floorSub)
      ));

    reset.whenPressed(new InstantCommand(driveSub::zeroGyroscope, driveSub));

    changeDrive.toggleWhenPressed(
      new RobotDriveCommand(
      () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
      () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
      () -> modifyAxis(driver.getRawAxis(4)) * DriveTrainSubsystems.maxAngularVelocityPerSecond,
      driveSub
    ));
  }

  public void resetOdo()
  {
    driveSub.resetOdometry(path.getInitialPose());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getDriveCommand() {
    // driveSub.resetOdometry(Robot.autoTrajectory.getInitialPose());

    
    // var transform = driveSubsystem.getCurrentPose().minus(exampleTrajectory.getInitialPose());
    // exampleTrajectory = straightTrajectory.transformBy(transform);
    // Transform2d transform = driveSub.getPose().minus(path.getInitialPose());
    // path.recalculateValues(transform, false);

    var thetaController = new ProfiledPIDController(Constants.ThetaController, 0, 0, Constants.thetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
    path,
    driveSub::getPose,
    Constants.m_kinematics,
    new PIDController(Constants.XController, 0, 0),
    new PIDController(Constants.YController, 0, 0),
    thetaController,
    driveSub::setModules,
    driveSub);


    // Run path following command, then stop at the end.
    return command.andThen(() -> driveSub.drive(new ChassisSpeeds(0,0,0))).beforeStarting(
      new SequentialCommandGroup(
        new InstantCommand(()-> thetaController.reset(0)),
        new InstantCommand(() -> driveSub.resetOdometry(path.getInitialPose()))
        ));
    // return command;

    // Run path following command, then stop at the end.
  }

  public Command getSequientialCommand() { 
    return new SequentialCommandGroup(
      new InstantCommand(() -> driveSub.zeroGyroscope()),
      new ParallelCommandGroup(
        new HomeLeftClimbCommand(leftClimb), 
        new HomeRightClimbCommand(rightClimb)),
      new TurnTurretCommand(turretSub),
      new StartTurretCommand(turretSub),
      getDriveCommand(),
      getAimCommand());
  }

  public Command getShootCommand() {
    return new ShootCommand(shooterSub).beforeStarting(new WaitCommand(6));
  }

  public Command getKickerCommand() {
    return new RunCommand(()-> kickSub.setKicker(-.8)).beforeStarting(new WaitCommand(7));
  }

  public Command getAimCommand() {
    return new ParallelCommandGroup(
      new  HoodCommand(hoodSub),
      new AimCommand(turretSub));
  }

  public Command getIntakeCommand() {
    return new ParallelCommandGroup(
      new RunCommand(
        ()-> intakeSub.setIntakeMotor(.9),
        intakeSub),
      new RunCommand(
        ()-> wallSub.setWall(.3),
        wallSub),
      new RunCommand(
        ()-> floorSub.setFloor(.3),
        floorSub)).beforeStarting(new WaitCommand(2.5));
  }


  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.1) {
        return (value - deadband) / (1.0 - deadband);
      } 
      else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.08);

    // Square the axis
    // value = Math.copySign(value * value, value);

    // SmartDashboard.putNumber("left x", driver.getLeftX());
    // SmartDashboard.putNumber("left y", driver.getLeftY());
    // SmartDashboard.putNumber("right x", driver.getRawAxis(4));

    return value;
  }
}

// this is where possible scrap code is stored

        // this is possible conditional command code
        // () -> driveSub.robotOrientedDrive(
        //   () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
        //   () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
        //   () -> modifyAxis(driver.getRawAxis(4)) * DriveTrainSubsystems.maxAngularVelocityPerSecond),
        // () -> driveSub.fieldOrientedDrive(
        //   () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
        //   () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
        //   () -> modifyAxis(driver.getRawAxis(4)) * DriveTrainSubsystems.maxAngularVelocityPerSecond),
        // true));