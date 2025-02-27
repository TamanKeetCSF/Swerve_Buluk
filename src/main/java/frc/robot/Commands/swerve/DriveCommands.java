package frc.robot.Commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.BulukLib.Swerve.SwerveConfig.measures;
import frc.robot.BulukLib.Util.QoLUtil;
import frc.robot.Subsystems.Drive.swerve;

import java.util.function.DoubleSupplier;



public class DriveCommands {

  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = QoLUtil.square(linearMagnitude);

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

   //Field relative drive command using two joysticks (controlling linear and angular velocities).
   
  public static Command joystickDrive(
      swerve drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          omega = Math.copySign(QoLUtil.square(omega), omega);

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command brake(swerve drive){
        return Commands.run(()->{
            
            drive.stopWithX();
            
        }, drive);
  }

  public static Command moveInX(swerve drive, double speed){
        return Commands.run(()-> {
            drive.runVelocity(new ChassisSpeeds(0, speed, 0));
        }, drive);
  }

  public static Command moveInY(swerve drive, double speed){
    return Commands.run(()-> {
        drive.runVelocity(new ChassisSpeeds(speed,0, 0));
    }, drive);
}

    public static Command testDrive(
      swerve drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double MAX_LINEAR_SPEED) {
    return Commands.run(
        () -> {
          // Apply deadband
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          omega = Math.copySign(QoLUtil.square(omega), omega);

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * MAX_LINEAR_SPEED,
                  linearVelocity.getY() * MAX_LINEAR_SPEED,
                  omega * MAX_LINEAR_SPEED / measures.DRIVE_BASE_RADIUS);
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runTestVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()), MAX_LINEAR_SPEED);
        },
        drive);
  }

    
}
  
