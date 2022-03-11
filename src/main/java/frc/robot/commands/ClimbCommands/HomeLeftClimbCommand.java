// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LeftClimbSubsytem;

public class HomeLeftClimbCommand extends CommandBase {
  /** Creates a new HomeLeftClimbCommand. */
  LeftClimbSubsytem leftClimb;
  public HomeLeftClimbCommand(LeftClimbSubsytem l) {
    // Use addRequirements() here to declare subsystem dependencies.
    leftClimb = l;
    addRequirements(l);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftClimb.set(-.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leftClimb.set(0);
    leftClimb.setPos();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftClimb.getVolt()>65;
  }
}
