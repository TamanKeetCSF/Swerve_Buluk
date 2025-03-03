  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ElevatorCommands.SetElevatorMin;
import frc.robot.Commands.IntakeCommands.SetIntakeComer;
import frc.robot.Commands.IntakeCommands.SetIntakeMax;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setComer extends SequentialCommandGroup {

  public setComer(Intake intake, Elevator elevator) {
    addCommands(
      new ParallelCommandGroup(
      new SetIntakeMax(intake),
      new SetElevatorMin(elevator))
      
      

    );
  }

}
