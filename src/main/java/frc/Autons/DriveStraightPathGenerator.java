package frc.Autons;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class DriveStraightPathGenerator {

    private final TrajectoryConfig config;
    private Trajectory trajectory;

    private final double trackWidthMeters = 0.5;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidthMeters);

    public DriveStraightPathGenerator(int distanceToTravel){
        config = new TrajectoryConfig(2, 1).setKinematics(kinematics);
        trajectory = TrajectoryGenerator.generateTrajectory(
            
            new Pose2d(0, 0, new Rotation2d(0)),

            List.of(new Translation2d(1, 0)),

            new Pose2d(distanceToTravel, 0, new Rotation2d(0)),

            config
        );
        
    }   

    public Trajectory getTrajectory(){
        return trajectory;
    }

}
