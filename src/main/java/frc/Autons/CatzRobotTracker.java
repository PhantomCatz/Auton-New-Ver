package frc.Autons;

import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotPositionSender;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import frc.Mechanisms.CatzDrivetrain;
import frc.Mechanisms.CatzSwerveModule;

public class CatzRobotTracker extends AbstractMechanism{

    //Singleton Instance
    private static final CatzRobotTracker robotTrackerInstance = new CatzRobotTracker();

    private final CatzDrivetrain driveTrain = CatzDrivetrain.getDrivetrainInstance();

    private Pose2d currentEstimatedPose;
    private SwerveDriveOdometry swerveOdometry;

    private static int ROBOT_TRACKER_THREAD_PERIOD_MS = 20;

    private CatzRobotTracker() {
        super(ROBOT_TRACKER_THREAD_PERIOD_MS);


        //Initializing robot position
        swerveOdometry = new SwerveDriveOdometry(
            CatzAutonomous.getSwervedrivekinematics(), Rotation2d.fromDegrees(driveTrain.getGyroAngle()),
            new SwerveModulePosition[] {
                getModulePosition(driveTrain.LT_FRNT_MODULE),
                getModulePosition(driveTrain.RT_FRNT_MODULE),
                getModulePosition(driveTrain.LT_BACK_MODULE),
                getModulePosition(driveTrain.RT_BACK_MODULE)
            }, new Pose2d());
    }


    public static CatzRobotTracker getRobotTrackerInstance(){
        return robotTrackerInstance;
    }

    //Method called in the CommandTranslator
    public void resetPosition(Pose2d pose){
        swerveOdometry.resetPosition(Rotation2d.fromDegrees(driveTrain.getGyroAngle()), new SwerveModulePosition[] {
            getModulePosition(driveTrain.LT_FRNT_MODULE),
            getModulePosition(driveTrain.RT_FRNT_MODULE),
            getModulePosition(driveTrain.LT_BACK_MODULE),
            getModulePosition(driveTrain.RT_BACK_MODULE)
        }, pose);
    }

    public Pose2d getCurrentEstimatedPose() {
        return currentEstimatedPose;
    }

    private void updateRobotPosition(){
        swerveOdometry.update(Rotation2d.fromDegrees(driveTrain.getGyroAngle()), new SwerveModulePosition[] {
            getModulePosition(driveTrain.LT_FRNT_MODULE),
            getModulePosition(driveTrain.RT_FRNT_MODULE),
            getModulePosition(driveTrain.LT_BACK_MODULE),
            getModulePosition(driveTrain.RT_BACK_MODULE)
        });
        
        currentEstimatedPose = swerveOdometry.getPoseMeters();
    }

    @Override
    public void update() {
        updateRobotPosition();
        RobotPositionSender.addRobotPosition(new RobotState(currentEstimatedPose));
    }
    
    private SwerveModulePosition getModulePosition(CatzSwerveModule module){
        double distanceMoved = (double) Units.inchesToMeters(module.getDriveDistance()); 

        return new SwerveModulePosition(
            distanceMoved, new Rotation2d(Math.toRadians(module.getAngle()))
        );
    }


    @Override
    public void smartDashboard() {
        
    }


    @Override
    public void smartDashboard_DEBUG() {
        
    }
}
