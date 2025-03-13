package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final SparkMax muneca = new SparkMax(Constants.idMotorConstants.ID_ArmMotor, MotorType.kBrushless);
  private final SparkMax Intake = new SparkMax(Constants.idMotorConstants.ID_Intake, MotorType.kBrushed);
  private RelativeEncoder encoderArm;
  private PIDController PIDMuneca;

  // PID Constants
  private static final double kP = 0.05;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  // Encoder and gearbox parameters
  private static final int NEO_TICKS_PER_REV = 4092;  // Ticks per revolution for the Neo internal encoder
  private static final int GEARBOX_RATIO = 48;          // Gearbox ratio
  private static final double TICKS_PER_REVOLUTION = NEO_TICKS_PER_REV / GEARBOX_RATIO;  // Total ticks per output revolution
  private static final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;

  double setPoint = 0;

  /** Creates a new Intake. */
  public Intake() {
    
    encoderArm = muneca.getEncoder();
    encoderArm.setPosition(0);
    
    //encoderArm.setPosition(0);  

    PIDMuneca = new PIDController(kP, kI, kD);
    PIDMuneca.setTolerance(2);
  }

  @Override
  public void periodic() {
    
  }

  public void setIntake(double power){
    muneca.set(power);
  }

  public void ponerAngulo(double angulo) {
    setPoint = angulo;
    PIDMuneca.setSetpoint(setPoint);
  }

  public void actualizarMotor() {
    double currentPositionTicks = encoderArm.getPosition();
    double pidOutput = PIDMuneca.calculate(currentPositionTicks);
    muneca.set(pidOutput);
    System.out.println("SetPoint (ticks): " + setPoint);
    System.out.println("Encoder Position (ticks): " + currentPositionTicks);
    System.out.println("Current Angle (deg): " + getArmAngle());
  }

  public double getArmAngle() {
    return encoderArm.getPosition(); // TICKS_PER_DEGREE;
  }

  public Command Comer(){
    //Intake.set(-0.23);
    return new InstantCommand(() -> Intake.set(-0.23), this);
  }
  public Command DesComer(){
    //Intake.set(0.68);
    return new InstantCommand(() -> Intake.set(0.68), this);
  }
  public Command DejarComer(){
    //Intake.set(-0.02);
    return new InstantCommand(() -> Intake.set(-0.02), this);
  }
}