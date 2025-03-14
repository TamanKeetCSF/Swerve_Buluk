// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DebuggingGroup;
import frc.robot.Commands.AutonomousCommands.AutonomoMain;
import frc.robot.Commands.AutonomousCommands.setPonerArriba;
import frc.robot.Commands.AutonomousCommands.setPonerAbajo;
import frc.robot.Commands.AutonomousCommands.setComer;
import frc.robot.Commands.ElevatorCommands.ManualSetElevator;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.HangingCommands.hangCommand;
import frc.robot.Commands.IntakeCommands.mantenerPosicion;
import frc.robot.Commands.swerve.AutoAlignAprilTag;
import frc.robot.Commands.swerve.DriveCommands;

import frc.robot.Subsystems.BallIntake;

import frc.robot.Subsystems.Hanger;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.Subsystems.Elevator;


public class RobotContainer {


  /////INICIALIZACION//////
  /// AQUI SE INICIALIZAN LOS SUBSISTEMAS QUE VA A LLAMAR EL ROBOT
  /// 
  swerve chassis = new swerve();
  CommandPS5Controller driver = new CommandPS5Controller(Constants.controlConstants.DriverControllerPort);
  private final PS5Controller Player1Controller = new PS5Controller(Constants.controlConstants.DriverControllerPort);
  private final XboxController Player2Controller = new XboxController(Constants.controlConstants.MechanismsControllerPort);
  private final Elevator m_elevador = new Elevator();
  private final Hanger m_colgador = new Hanger();
  private final BallIntake m_bola = new BallIntake();
  public final Intake m_intake = new Intake();
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    
  ////AQUI LLAMAMOS A LAS INTRUCCIONES QUE SEGUIRA EL ROBOT TODO EL TIEMPO DURANTE EL TELE OP
  /// 
    NamedCommands.registerCommand("ponerAbajo", new setPonerAbajo(m_intake, m_elevador));
    NamedCommands.registerCommand("goToComer", new setComer(m_intake, m_elevador));
    NamedCommands.registerCommand("Comer", new InstantCommand(()-> m_intake.Comer()));
    NamedCommands.registerCommand("DesComer", new InstantCommand(()-> m_intake.DesComer()));
    NamedCommands.registerCommand("DejarComer", new InstantCommand(()-> m_intake.DejarComer()));
    autoChooser = AutoBuilder.buildAutoChooser("a");
    configureBindings();
    //chassis.setDefaultCommand(new AutoAlignAprilTag(chassis));
    chassis.setDefaultCommand(

    
      DriveCommands.joystickDrive(
        chassis,
        ()-> driver.getLeftY() * 0.7,
        ()-> driver.getLeftX() * 0.7,
        ()-> driver.getRightX() * 0.7,
        ()-> Player1Controller.getR2Axis(),
        ()-> Player1Controller.getL2Axis()));

    driver.L1().whileTrue(DriveCommands.brake(chassis));
    m_elevador.setDefaultCommand(new ManualSetElevator(m_elevador, Player2Controller));
    m_intake.setDefaultCommand(new mantenerPosicion(m_intake, Player2Controller));
    m_colgador.setDefaultCommand(new hangCommand(m_colgador, Player2Controller));
  }

  private void configureBindings() {
        // declaracion control 1

      final JoystickButton button1Start = new JoystickButton(Player1Controller, 8);
      final JoystickButton button1Select = new JoystickButton(Player1Controller, 7);
      final JoystickButton button1A = new JoystickButton(Player1Controller, 1);
      final JoystickButton button1B = new JoystickButton(Player1Controller, 2);
      final JoystickButton button1X = new JoystickButton(Player1Controller, 3);
      final JoystickButton button1Y = new JoystickButton(Player1Controller, 4);

    //declaracion control2
      final JoystickButton button2A = new JoystickButton(Player2Controller, 1);
      final JoystickButton button2B = new JoystickButton(Player2Controller, 2);
      final JoystickButton button2X = new JoystickButton(Player2Controller, 3);
      final JoystickButton button2Y = new JoystickButton(Player2Controller, 4);
      final JoystickButton button2Select = new JoystickButton(Player2Controller, 7);
      final JoystickButton button2Start = new JoystickButton(Player2Controller, 8);

      final JoystickButton button2BumperL = new JoystickButton(Player2Controller, 5);
      final JoystickButton button2BumperR = new JoystickButton(Player2Controller, 6);
      Trigger leftTrigger = new Trigger(() -> Player2Controller.getRawAxis(2) > 0.4); // Left trigger
      Trigger rightTrigger = new Trigger(() -> Player2Controller.getRawAxis(3) > 0.4); // Right trigger

          //bindings subsistemas
//Swerves
    button1A.whileTrue(new AutoAlignAprilTag(chassis));
      //intake

       //button2Y.onTrue(new DebuggingGroup(m_intake,m_elevador)); 
       //button2A.onTrue(new setPonerAbajo(m_intake, m_elevador)); 
       //button2B.onTrue(new setPonerArriba(m_intake, m_elevador)); 
       button2B.onTrue(new setPonerArriba(m_intake, m_elevador)); 
       button2X.onTrue(new setPonerAbajo(m_intake, m_elevador)); 
       button2A.onTrue(new setComer(m_intake, m_elevador)); 


      rightTrigger.onTrue(new InstantCommand(()-> m_intake.Comer()))
      .onFalse(new InstantCommand(()-> m_intake.DejarComer()));

      leftTrigger.onTrue(new InstantCommand(()-> m_intake.DesComer()))
      .onFalse(new InstantCommand(()-> m_intake.DejarComer()));

      //elevator
      new POVButton(Player2Controller, 0).onTrue(new InstantCommand(() -> m_bola.marcoBaja()))
      .onFalse(new InstantCommand(() -> m_bola.marcoStop()));
      new POVButton(Player2Controller, 180).onTrue(new InstantCommand(() -> m_bola.marcoSube()))
      .onFalse(new InstantCommand(() -> m_bola.marcoStop()));;

      //ball intake
      button2BumperR.onTrue(new InstantCommand(() -> m_bola.ballIntakeComer(),m_bola))
      .onFalse(new InstantCommand(() -> m_bola.ballIntakeStop(),m_bola));

      button2BumperL.onTrue(new InstantCommand(() -> m_bola.ballIntakeSacar(),m_bola))
      .onFalse(new InstantCommand(() -> m_bola.ballIntakeStop(),m_bola));

      //colgar
      
      //Swerve Drive Bindings
    
   
  }

  public Command getAutonomousCommand() {
    System.out.println("funciono");
    return autoChooser.getSelected();
    //return new AutonomoMain(chassis, m_intake);
  }

}