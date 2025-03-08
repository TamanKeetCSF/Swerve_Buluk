package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;
import frc.robot.BulukLib.Swerve.SwerveConfig;
import frc.robot.BulukLib.Swerve.SwerveConfig.currentLimiting;
import frc.robot.BulukLib.Swerve.SwerveConfig.measures;
import frc.robot.BulukLib.Swerve.SwerveConfig.reductions;
import frc.robot.BulukLib.Swerve.SwerveConfig.speeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Drive.ModuleSpark;

public class swerve extends SubsystemBase {

    private final Pigeon2 gyro = new Pigeon2(7);
    private static final double MAX_LINEAR_SPEED = Units.feetToMeters(19.0);
    private static final double TRACK_WIDTH_X = SwerveConfig.measures.TRACK_WIDTH_X; 
    private static final double TRACK_WIDTH_Y = SwerveConfig.measures.TRACK_WIDTH_Y; 
    private static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(),
        new SwerveModulePosition(), new SwerveModulePosition()
    };
    private final SwerveDrivePoseEstimator poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    ModuleSpark[] modules = new ModuleSpark[4];

    public swerve() {
        // Instantiate modules using your ModuleSpark class
        modules[0] = new ModuleSpark(0);
        modules[1] = new ModuleSpark(1);
        modules[2] = new ModuleSpark(2);
        modules[3] = new ModuleSpark(3);

        // Configure gyro
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0.0);

        // *** NEW: Configure AutoBuilder for PathPlanner ***
        try {
            // Optionally load a RobotConfig from the GUI
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,                // Pose supplier
                this::resetPose,              // Reset pose method
                this::getSpeeds,              // Chassis speeds supplier (robot-relative)
                this::driveRobotRelative,     // Drive robot relative method
                new PPHolonomicDriveController(
                    new PIDConstants(0.05, 0, 0),
                    new PIDConstants(5.0, 0, 0)
                ),
                config,                       // Robot configuration (loaded from GUI settings)
                () -> {
                    // Flip the path if the alliance is Red.
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this                          // Reference to this subsystem (for command requirements)
            );
        } catch(Exception e) {
            DriverStation.reportError("Failed to configure AutoBuilder", e.getStackTrace());
        }

        // Optionally, put a Field2d on the dashboard for visualization (if you have one)
        SmartDashboard.putData("Field", new edu.wpi.first.wpilibj.smartdashboard.Field2d());
    }

    @Override
    public void periodic() {
        // Update each module periodically
        for (var module : modules) {
            module.periodic();
        }
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log module measurements
        double[] meters = new double[4];
        double[] metersPerSec = new double[4];
        double[] metersTest = new double[4];
        double[] ositos = new double[4];
      
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

        // Compute module positions and update odometry
        SwerveModulePosition[] modulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            moduleDeltas[i] = new SwerveModulePosition(
                modulePositions[i].distanceMeters - lastModulePositions[i].distanceMeters,
                modulePositions[i].angle);
            lastModulePositions[i] = modulePositions[i];
        }

        // Update gyro reading: if connected use the Pigeon2 reading, otherwise integrate module deltas.
        if (gyro.isConnected()) {
            rawGyroRotation = getPigeonRotation();
        } else {
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }
        SmartDashboard.putNumber("Pigeon2", getAngle());

        // Update the odometry based on current gyro and module positions.
        poseEstimator.update(rawGyroRotation, modulePositions);
    }

    public double getAngle() {
        return -gyro.getYaw().getValueAsDouble();
    }

    public Rotation2d getPigeonRotation() {
        return Rotation2d.fromDegrees(-getAngle());
    }

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            SwerveConstants.FRONT_LEFT_POSITION,
            SwerveConstants.FRONT_RIGHT_POSITION,
            SwerveConstants.BACK_LEFT_POSITION,
            SwerveConstants.BACK_RIGHT_POSITION
        };
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    // Command the robot to drive with a given velocity.
    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }
    }

    public void runTestVelocity(ChassisSpeeds speeds, double maxSPEED) {
        ChassisSpeeds disc = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(disc);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSPEED);
        for (int i = 0; i < 4; i++) {
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
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    // --- Methods for PathPlanner integration ---
    // Return the current estimated pose.
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // Reset odometry with a new starting pose.
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    // Return current chassis speeds from module states.
    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    // Drive using robot-relative speeds. Here we convert field-relative speeds to robot-relative.
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeSpeeds, getPose().getRotation());
        runVelocity(fieldRelative);
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

    // Returns maximum linear speed in meters per second.
    public double getMaxLinearSpeedMetersPerSec() {
        return 5.2;
    }

    // Returns maximum angular speed in radians per second.
    public double getMaxAngularSpeedRadPerSec() {
        return MAX_ANGULAR_SPEED;
    }

    // For vision-based updates: add a vision observation with a standard deviation.
    public void addObservationStd(Pose2d pose, double timeStamps, Matrix<N3, N1> std) {
        poseEstimator.addVisionMeasurement(pose, timeStamps, std);
    }

    // For vision-based updates: add a vision observation without standard deviation.
    public void addObservation(Pose2d pose, double timeStamps) {
        poseEstimator.addVisionMeasurement(pose, timeStamps);
    }
}
