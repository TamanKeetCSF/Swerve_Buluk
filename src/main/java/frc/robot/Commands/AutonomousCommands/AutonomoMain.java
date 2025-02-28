// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands;

import java.security.Timestamp;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.IntakeCommands.SetIntakeComer;
import frc.robot.Commands.swerve.AutoDrive;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Drive.swerve;

public class AutonomoMain extends SequentialCommandGroup {
  public AutonomoMain(swerve drive, Intake intake){
    addCommands(
      new InstantCommand(() -> drive.runVelocity(new ChassisSpeeds(-0.6,0,0)), drive ),
      new WaitCommand(3.5),
      new SetIntakeComer(intake),
      new InstantCommand(() -> drive.runVelocity(new ChassisSpeeds(0,0,0)), drive )
      //new AutoDrive(drive,0.0)

    );
    
  }

}
