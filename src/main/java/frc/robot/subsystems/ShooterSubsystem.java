// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  public TalonSRX motor;
  public DigitalInput breakbeam;
  public int rpm = 0;

  public ShooterSubsystem(TalonSRX motor, DigitalInput breakbeam) {
    this.motor = motor;
    this.breakbeam = breakbeam;
    motor.configFactoryDefault();
  }

  public void runMotors(int rpm) {
      motor.set(ControlMode.Velocity, rpm / 60 * 2048);
      this.rpm = rpm;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RPM", rpm);
  }

  public boolean getBreakBeam() {
    return breakbeam.get();
  }


}
