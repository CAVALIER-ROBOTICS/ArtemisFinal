// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RightClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  CANSparkMax rightClimb = new CANSparkMax(Constants.rightClimb,MotorType.kBrushless);
  RelativeEncoder rightEnc = rightClimb.getEncoder();

  public RightClimbSubsystem() {
    // rightClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65300);
    // rightClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65350);
    // leftClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65400);
    // leftClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65460);
    // rightClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65360);
  }

  public void setClimb(DoubleSupplier x) {
    rightClimb.set(-x.getAsDouble());
  }

  public void set(double x)
  {
    rightClimb.set(x);
  }
  public double getVolt() {
    return rightClimb.getOutputCurrent();
  }

  public void setPos() {
    rightEnc.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RightClimb Enc", rightEnc.getPosition());
    SmartDashboard.putNumber("right climb Volts", getVolt());
  }
}
