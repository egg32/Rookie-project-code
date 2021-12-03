// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RunMotor extends CommandBase {

  public ShooterSubsystem shooter;
  public double time;

  /** Creates a new RunMotor. */
  public RunMotor(TalonSRX motor, DigitalInput breakbeam) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = new ShooterSubsystem(motor, breakbeam);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!shooter.getBreakBeam()) {
      time = Timer.getFPGATimestamp();
    }
    if (Timer.getFPGATimestamp() - time >= 3) {
      return true;
    }

    return false;
  }
}
