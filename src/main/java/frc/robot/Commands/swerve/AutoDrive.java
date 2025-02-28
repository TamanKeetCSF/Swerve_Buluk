// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.swerve;

import java.security.Timestamp;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Drive.swerve;

public class AutoDrive extends Command {
  private final swerve drive;
  private final double speed;
  /** Creates a new AutonomoMain. */
  public AutoDrive(swerve drive, double speed) {
    this.drive = drive;
    this.speed = speed;
    addRequirements(drive);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
          System.out.println(speed);
          drive.runVelocity(new ChassisSpeeds(speed,0,0));
        }
    
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // drive.stopWithX();
    return false;
  }
}
