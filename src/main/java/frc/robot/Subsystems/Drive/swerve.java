package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;
import frc.robot.BulukLib.Swerve.SwerveConfig;
import frc.robot.BulukLib.Swerve.SwerveConfig.currentLimiting;
import frc.robot.BulukLib.Swerve.SwerveConfig.measures;
import frc.robot.BulukLib.Swerve.SwerveConfig.reductions;
import frc.robot.BulukLib.Swerve.SwerveConfig.speeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import java.util.HashMap;
import java.util.Map;

public class swerve extends SubsystemBase {

    private Pigeon2 gyro = new Pigeon2(7);

    private static final double MAX_LINEAR_SPEED = Units.feetToMeters(19.0);
    private static final double TRACK_WIDTH_X = SwerveConfig.measures.TRACK_WIDTH_X; 
    private static final double TRACK_WIDTH_Y = SwerveConfig.measures.TRACK_WIDTH_Y; 
    private static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
    private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    ModuleSpark []modules  = new ModuleSpark[4];

    public swerve(){

     
        modules[0] = new ModuleSpark(0);
        modules[1] = new ModuleSpark(1);
        modules[2] = new ModuleSpark(2);
        modules[3] = new ModuleSpark(3);

        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0.0);
               // Configure AutoBuilder using the overload without DriveFeedforwards
               AutoBuilder.configure(
                this::getPose,                          // Robot pose supplier
                this::resetPose,                        // Odometry reset method
                this::getRobotRelativeSpeeds,           // Robot-relative chassis speeds supplier
                (ChassisSpeeds speeds) -> {             // Drive output lambda (only one parameter)
                    runVelocity(speeds);
                },
                new PPHolonomicDriveController(         // Path-following controller
                    new PIDConstants(5.0, 0.0, 0.0),      // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0)       // Rotation PID constants
                ),
                getPathPlannerConfiguration(),          // Robot configuration
                () -> {                                 // Alliance mirroring supplier
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == Alliance.Red;
                },
                this                                    // Subsystem requirements
            );
    }

    double [] meters = new double[4];

    double[] metersPerSec = new double[4];

    double [] metersTest = new double[4];

    double [] ositos = new double[4];
  
    public void periodic(){

      for (var module : modules) {
        module.periodic();
      }
      if (DriverStation.isDisabled()) {
        for (var module : modules) {
          module.stop();
      }}


      meters[0] = modules[0].getDrivePositionMeters();
      meters[1] = modules[1].getDrivePositionMeters();
      meters[2] = modules[2].getDrivePositionMeters();
      meters[3] = modules[3].getDrivePositionMeters();

      metersPerSec[0] = modules[0].getRotorMPS();
      metersPerSec[1] = modules[1].getRotorMPS();
      metersPerSec[2] = modules[2].getRotorMPS();
      metersPerSec[3] = modules[3].getRotorMPS();

      metersTest[0] = modules[0].getRotorMeters();
      metersTest[1] = modules[1].getRotorMeters();
      metersTest[2] = modules[2].getRotorMeters();
      metersTest[3] = modules[3].getRotorMeters();

      ositos[0] = modules[0].getDriveVelocityMetersxSec();
      ositos[1] = modules[1].getDriveVelocityMetersxSec();
      ositos[2] = modules[2].getDriveVelocityMetersxSec();
      ositos[3] = modules[3].getDriveVelocityMetersxSec();

      SmartDashboard.putNumberArray("METERS_MODULES_FIRST", meters);

      SmartDashboard.putNumberArray("METERS_MODULES_CONVERT", metersTest);

      SmartDashboard.putNumberArray("METERSPERSECOND", metersPerSec);

      SmartDashboard.putNumberArray("OSITOS", ositos);


      SwerveModulePosition[] modulePositions = getModulePositions();
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyro.isConnected() == true) {
        // Use the real gyro angle
        rawGyroRotation = getPigeonRotation();
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
      SmartDashboard.putNumber("Pigeon2", getAngle());

      // Apply odometry update
      poseEstimator.update(rawGyroRotation, modulePositions);
    }

    public double getAngle(){
      return -gyro.getYaw().getValueAsDouble();
    }

    public Rotation2d getPigeonRotation(){
      return Rotation2d.fromDegrees(-getAngle());
    }

    public ChassisSpeeds getChassisSpeeds(){
      return kinematics.toChassisSpeeds(getModuleStates());
    }
    
    // Returns robot-relative chassis speeds (assumed to equal getChassisSpeeds)
    public ChassisSpeeds getRobotRelativeSpeeds() {
      return getChassisSpeeds();
  }

    public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      SwerveConstants.FRONT_LEFT_POSITION,
      SwerveConstants.FRONT_RIGHT_POSITION,
      SwerveConstants.BACK_LEFT_POSITION,
      SwerveConstants.BACK_RIGHT_POSITION
      };
    }

  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }


  }

 
  public void runTestVelocity(ChassisSpeeds speeds, double maxSPEED){
    ChassisSpeeds disc = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(disc);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSPEED);

    for(int i = 0; i < 4; i ++){
      modules[i].runTestSetpoint(states[i]);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
  
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public Double getX() {
    return getPose().getX();
  }

  public Double getY() {
    return getPose().getY();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return 5.2;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  public void addObservationStd(Pose2d pose, double timeStamps, Matrix< N3, N1> std){
      poseEstimator.addVisionMeasurement(pose, timeStamps, std);
  }
  public void addObservation(Pose2d pose, double timeStamps){
    poseEstimator.addVisionMeasurement(pose, timeStamps);
  }
     // Returns the Robot configuration for PathPlanner.
  public RobotConfig getPathPlannerConfiguration() {
    return new RobotConfig(
      measures.robotMassKg,
      measures.robotMOI,
      new ModuleConfig(
          measures.WHEELRADIUS,
          getMaxLinearSpeedMetersPerSec(),
          1.0,
          DCMotor.getNEO(1).withReduction(reductions.DriveReduction),
          currentLimiting.driveCurrentLimit,
          1
      ),
      getModuleTranslations()
  );
  }
}
