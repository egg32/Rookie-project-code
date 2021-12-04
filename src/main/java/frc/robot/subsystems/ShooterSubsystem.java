// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private TalonSRX motor;
  private DigitalInput breakbeam;

  private double startTime;
  private boolean ballInSystem;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    motor = new TalonSRX(22);
    breakbeam = new DigitalInput(0);

    startTime = Timer.getFPGATimestamp();
    ballInSystem = false;

    motor.configFactoryDefault();
  }

  public void runMotors(double output) {
      motor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() { 
    if(!breakbeam.get()) {
      ballInSystem = true;

      runMotors(0.3);
    }
      
    if(!ballInSystem)
      startTime = Timer.getFPGATimestamp();

    if(Timer.getFPGATimestamp() - startTime >= 3) {
      ballInSystem = false;

      runMotors(0);
    }
  }

  public DigitalInput getBreakbeam() {
    return breakbeam;
  }
}
