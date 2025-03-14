// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.AutonomousCommands.setComer;
import frc.robot.Commands.AutonomousCommands.setPonerAbajo;
import frc.robot.Commands.AutonomousCommands.setPonerArriba;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DebuggingGroup extends SequentialCommandGroup {
  /** Creates a new DebuggingGroup. */
  public DebuggingGroup(Intake intake, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> intake.Comer()),
      new setComer(intake, elevator),
      new WaitCommand(1),
      new InstantCommand(()-> intake.DesComer()),
      new setPonerArriba(intake, elevator),
      new WaitCommand(2),
      new setPonerAbajo(intake, elevator),
      new WaitCommand(1),
      new InstantCommand(()-> intake.DejarComer())
    );
  }
}
