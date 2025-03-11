package frc.robot.Commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetIntakeComer extends Command {

  private final Intake intake;
  /** Creates a new SetIntakeMax. */
  public SetIntakeComer(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int signDif =(intake.getArmAngle()-Constants.OperatorConstants.DesiredComer > 0)?-1:1;
    intake.setIntake(OperatorConstants.MaxArmSpeed*signDif);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntake(0.02);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.IsIntakeComerDesired();
  }
}