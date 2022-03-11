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

public class LeftClimbSubsytem extends SubsystemBase {
  /** Creates a new LeftClimbSubsytem. */
  CANSparkMax leftClimb = new CANSparkMax(Constants.leftClimb,MotorType.kBrushless);
  RelativeEncoder leftEnc = leftClimb.getEncoder();

  public LeftClimbSubsytem() {
    // leftClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65410);
  }

  public double getVolt() {
    return leftClimb.getOutputCurrent();
  }

  public void setClimb(DoubleSupplier x) {
    leftClimb.set(-x.getAsDouble());
  }

  public void set(double x)
  {
    leftClimb.set(x);
  }

  public void setPos() {
    leftEnc.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("leftClimb Enc", leftEnc.getPosition());
    SmartDashboard.putNumber("leftClimb Volts", getVolt());


  }
}
