package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class mantenerPosicion extends Command {

  private final Intake intake;
  private final XboxController control;

  /** Creates a new ManualSetIntake. */
  public mantenerPosicion(Intake intake, XboxController control) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.control = control;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double potencia = control.getRightX();

    if (potencia <= 0.3 && potencia >= -0.3){
      intake.actualizarMotor();
    }
    
    else if ((intake.getArmAngle() > Constants.OperatorConstants.MinArmPosition && potencia <= -0.3) || (intake.getArmAngle() < Constants.OperatorConstants.MaxArmPosition && potencia >= 0.3)){
      intake.setIntake(potencia * 0.4);
      intake.ponerAngulo(intake.getArmAngle());
      
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntake(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
