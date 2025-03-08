package frc.robot.Commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.swerve;

public class AutoAlignAprilTag extends Command {
    private final swerve drive;
    
    // Controladores PID 
    private final PIDController rotationController = new PIDController(0.05, 0.0, 0.0);
    private final PIDController forwardController = new PIDController(1.5, 1.15, 1.4);
    
    // Angulos de la limelight
    private static final double LIMELIGHT_MOUNTING_ANGLE = Math.toRadians(30.0); // Angulo de montado de la camara en radianes
    private static final double LIMELIGHT_HEIGHT = 0.67;   // Camara  altura en metros
    private static final double TARGET_HEIGHT = 0.5;      // AprilTag altura en metros
    
    // distancia deseada atras del limelight
    private static final double DESIRED_DISTANCE = 4 * 0.0254; 
    
    // tolerancias
    private static final double DISTANCE_TOLERANCE = 0.0254; 
    private static final double ANGLE_TOLERANCE = 1.0;        
    
    public AutoAlignAprilTag(swerve drive) {
        this.drive = drive;
        addRequirements(drive);
        
        rotationController.setTolerance(ANGLE_TOLERANCE);
        forwardController.setTolerance(DISTANCE_TOLERANCE);
    }
    
    @Override
    public void initialize() {
        rotationController.reset();
        forwardController.reset();
    }
    
    @Override
    public void execute() {
        // los valores de la limelight 3
        var table = NetworkTableInstance.getDefault().getTable("limelight");
        double tv = table.getEntry("tv").getDouble(0.0);
        
        // deteccion del target
        if (tv < 1.0) {
            drive.runVelocity(new ChassisSpeeds(0, 0, 0));
            return;
        }
        
        double tx = table.getEntry("tx").getDouble(0.0); // offset horizontal (grados)
        double ty = table.getEntry("ty").getDouble(0.0); // offset vertical (grados))

        System.out.println("tx "+ tx);
        System.out.println("ty "+ ty);
        
        // Computo de la distancia usando esta formula que encontre en chiefdelphi:
        // distance = (targetHeight - cameraHeight) / tan(cameraMountingAngle + ty)
        double tyRadians = Math.toRadians(ty);
        double distance = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_MOUNTING_ANGLE + tyRadians);
        
        // Calculo error
        double distanceError = distance - DESIRED_DISTANCE;
        double rotationError = tx; // Queremos que tx en grados sea 0
        
        // Calculo salida PID 
        double forwardOutput = forwardController.calculate(distanceError, 0.0);
        double rotationOutput = rotationController.calculate(rotationError, 0.0);
        
        // Manejo robot, hacia delante o atras (x) y rotacion (z).
        // Se asume que el robot ya esta viendo hacia el objetivo
        drive.runVelocity(new ChassisSpeeds(-forwardOutput, 0, -rotationOutput));
    }
    
    @Override
    public boolean isFinished() {
        // se termina cuando los valores ya esten dentro de la tolerancia
        return forwardController.atSetpoint() && rotationController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}