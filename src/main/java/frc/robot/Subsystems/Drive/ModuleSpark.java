package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.BulukLib.Math.rotorConversions;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl.ClosedLoopRequest;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl.OutputType;
import frc.robot.BulukLib.MotionControllers.Gains.FeedForwardGains;
import frc.robot.BulukLib.MotionControllers.Gains.PIDGains;
import frc.robot.BulukLib.MotionControllers.ModuleController.PIDFController;
import frc.robot.BulukLib.Swerve.SwerveConfig;
import frc.robot.BulukLib.Swerve.SwerveConfig.measures;
import frc.robot.BulukLib.Swerve.SwerveConfig.reductions;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.backLeft;
import frc.robot.Constants.DriveConstants.backRight;
import frc.robot.Constants.DriveConstants.frontLeft;
import frc.robot.Constants.DriveConstants.frontRight;

public class ModuleSpark {
    SparkMax driveSparkMax, turnSparkMax;
    RelativeEncoder enc_drive, enc_turn;

    CANcoder AbsoluteEncoder;

    boolean isDriveMotorInverted;
    boolean isTurnMotorInverted;

    SparkMaxConfig config_drive, config_turn;

    private final ClosedLoopControl turnControl;
    private final ClosedLoopRequest turnRequest;
    private final PIDController drivePID;
    private final PIDFController speedController; // test
    private final SimpleMotorFeedforward drivFeedforward;
    private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop

    public ModuleSpark(int index){

        config_drive = new SparkMaxConfig();
        config_turn = new SparkMaxConfig();

        drivePID = new PIDController(0.05, 0.0, 0.0);
        drivFeedforward = new SimpleMotorFeedforward(0.1, 0.08);
    
        turnControl = new ClosedLoopControl(DriveConstants.turnGains, OutputType.kPositive);
        turnRequest =  turnControl.new ClosedLoopRequest();

        turnControl.enableContinuousInput(Math.PI);
        turnControl.initTuning("TurnTune");

        speedController = new PIDFController(new PIDGains(0.05, 0.0, 0.0),
                new FeedForwardGains(0.1, 0.12, 0), OutputType.kPositive);

        switch (index) {
          case 0:
            driveSparkMax = new SparkMax(frontLeft.DrivePort, MotorType.kBrushless);
            turnSparkMax = new SparkMax(frontLeft.TurnPort, MotorType.kBrushless);
            AbsoluteEncoder = new CANcoder(frontLeft.EncPort);
            isDriveMotorInverted = frontLeft.DrivemotorReversed;
            isTurnMotorInverted = frontLeft.TurnmotorReversed;
            break;
          case 1:
            driveSparkMax = new SparkMax(frontRight.DrivePort, MotorType.kBrushless);
            turnSparkMax = new SparkMax(frontRight.TurnPort, MotorType.kBrushless);
            AbsoluteEncoder = new CANcoder(frontRight.EncPort);
            isDriveMotorInverted = frontRight.DrivemotorReversed;
            isTurnMotorInverted = frontRight.TurnmotorReversed;
            break;
          case 2:
            driveSparkMax = new SparkMax(backLeft.DrivePort, MotorType.kBrushless);
            turnSparkMax = new SparkMax(backLeft.TurnPort, MotorType.kBrushless);
            AbsoluteEncoder = new CANcoder(backLeft.EncPort);
            isDriveMotorInverted = backLeft.DrivemotorReversed;
            isTurnMotorInverted = backLeft.TurnmotorReversed;
            break;
          case 3:
            driveSparkMax = new SparkMax(backRight.DrivePort, MotorType.kBrushless);
            turnSparkMax = new SparkMax(backRight.TurnPort, MotorType.kBrushless);
            AbsoluteEncoder = new CANcoder(backRight.EncPort);
            isDriveMotorInverted = backRight.DrivemotorReversed;
            isTurnMotorInverted = backRight.TurnmotorReversed;
            break;
          default:
            throw new RuntimeException("Invalid module index");
        }

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        enc_drive = driveSparkMax.getEncoder();
        enc_drive.setPosition(0.0);
        enc_turn = driveSparkMax.getEncoder();

        config_drive();
        config_turn();
    }

    public void config_drive(){
        config_drive
        .inverted(isDriveMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(SwerveConfig.currentLimiting.driveCurrentLimit)
        .voltageCompensation(12);
  
        config_drive.encoder
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);
  
        driveSparkMax.configure(config_drive, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  
    public void config_turn(){
        config_turn
        .inverted(isTurnMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(SwerveConfig.currentLimiting.turnCurrentLimit)
        .voltageCompensation(12);
  
        config_turn.encoder
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);

        turnSparkMax.configure(config_turn, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic(){
        if (angleSetpoint != null) {
            turnSparkMax.setVoltage(
                turnControl.runRequest(
                    turnRequest.withReference(AngleEncoder().getRadians())
                    .toSetpoint(angleSetpoint.getRadians()))
            );
      
            if (speedSetpoint != null) {
              double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnControl.getCurrentError());
              double velocityRadPerSec = adjustSpeedSetpoint / SwerveConfig.measures.WHEELRADIUS;
              driveSparkMax.setVoltage(
                  drivFeedforward.calculate(velocityRadPerSec)
                      + drivePID.calculate(Units.rotationsPerMinuteToRadiansPerSecond(enc_drive.getVelocity()) / SwerveConfig.reductions.DriveReduction, velocityRadPerSec)
              );
            }
        }
    }

    public void runTestSetpoint(SwerveModuleState state){
        state.optimize(AngleEncoder());
        double adjustSpeed = state.speedMetersPerSecond * Math.cos(turnControl.getCurrentError());
        double wheelVelocity = rotorConversions.metersPerSecToWheelRadPerSec(adjustSpeed, measures.WHEELRADIUS);
        turnSparkMax.setVoltage(
          turnControl.runRequest(
            turnRequest.withReference(AngleEncoder().getRadians())
            .toSetpoint(angleSetpoint.getRadians())
          )
        );
        driveSparkMax.setVoltage(
          speedController.calculate(rotorConversions.metersPerSecToWheelRadPerSec(getRotorMPS(), measures.WHEELRADIUS), wheelVelocity)
        );
    }

    public double getRotorMPS(){
      return rotorConversions.rotorRPMtoMetersPerSec(enc_drive.getVelocity(), reductions.DriveReduction, measures.WHEELRADIUS);
    }

    public double getRotorMeters(){
      return rotorConversions.rotorRotationsToMeters(enc_drive.getPosition(), reductions.DriveReduction, measures.WHEELRADIUS);
    }
  
    public Rotation2d AngleEncoder(){
      return new Rotation2d(Units.rotationsToRadians(AbsoluteEncoder.getAbsolutePosition().getValueAsDouble()));
    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
      state.optimize(AngleEncoder());
      angleSetpoint = state.angle;
      speedSetpoint = state.speedMetersPerSecond;
      return state;
    }

    public void setSpeed(SwerveModuleState desiredState){
      driveSparkMax.setVoltage(desiredState.speedMetersPerSecond);
    }

    public double getDrivePositionMeters(){
      return Units.rotationsToRadians(enc_drive.getPosition() / SwerveConfig.reductions.DriveReduction)
              * SwerveConfig.measures.WHEELRADIUS;
    }

    public double getDriveVelocityMetersxSec(){
      return Units.rotationsPerMinuteToRadiansPerSecond(enc_drive.getVelocity()) / SwerveConfig.reductions.DriveReduction * SwerveConfig.measures.WHEELRADIUS;
    }
    
    public SwerveModulePosition getPosition(){
      return new SwerveModulePosition(getDrivePositionMeters(), AngleEncoder());
    }
    
    public SwerveModuleState getState(){
      return new SwerveModuleState(getDriveVelocityMetersxSec(), AngleEncoder());
    }

    public void stop() {
      driveSparkMax.set(0.0);
      turnSparkMax.set(0.0);
      angleSetpoint = null;
      speedSetpoint = null;
    }
}

