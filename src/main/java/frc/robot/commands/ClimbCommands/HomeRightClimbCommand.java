// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RightClimbSubsystem;

public class HomeRightClimbCommand extends CommandBase {
  /** Creates a new HomeRightClimbCommand. */
  RightClimbSubsystem rightClimb;

  public HomeRightClimbCommand(RightClimbSubsystem r) {
    // Use addRequirements() here to declare subsystem dependencies.
    rightClimb = r;
    addRequirements(r);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rightClimb.set(-.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rightClimb.set(0);
    rightClimb.setPos();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rightClimb.getVolt()>65;
  }
}
