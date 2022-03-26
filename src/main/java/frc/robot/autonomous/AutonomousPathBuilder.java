package frc.robot.autonomous;

import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public final class AutonomousPathBuilder {

    public static HashMap<String, Trajectory> assemblePaths() {

        HashMap<String, Trajectory> trajectories = new HashMap<>();

        var files = Filesystem.getDeployDirectory().listFiles(new FilenameFilter() {
            @Override
            public boolean accept(File dir, String name) {
                return name.endsWith(".wpilib.json");
            }
        });

        for (File file : files) {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(file.toPath());
            try {
                var trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                trajectories.put(
                    getBareFileName(file.getName()),
                    trajectory);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open file: " + file, ex.getStackTrace());
            }
        }

        return trajectories;
    }

    private static String getBareFileName(String file) {
        int end = file.indexOf(".");
        
        if (end == -1) {
            return file;
        }
        else {
            return file.substring(0, end);
        }
    }
}
