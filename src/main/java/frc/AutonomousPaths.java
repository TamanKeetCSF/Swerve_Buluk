package frc.robot.Commands.swerve;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.swerve;

import java.util.List;

public class AutonomousPaths {
    public static Command getAutonomousCommand(swerve drive) {
        // Load the trajectory group from PathPlanner.
        // "ExamplePath" should be the name of the path you designed in the PathPlanner GUI.
        //List<PathPlannerTrajectory> trajectoryGroup = 
            //PathPlanner.loadPathGroup("ExamplePath", drive.getMaxLinearSpeedMetersPerSec(), drive.getMaxAngularSpeedRadPerSec());
        // Build the full autonomous command using the AutoBuilder defined in the swerve subsystem.
        //return drive.getAutoBuilder().fullAuto(trajectoryGroup);
    }
}
