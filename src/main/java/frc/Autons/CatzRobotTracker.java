package frc.Autons;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import frc.Mechanisms.CatzDrivetrain;
import frc.Mechanisms.CatzSwerveModule;

public class CatzRobotTracker extends ThreadRunner{

    //Singleton Instance
    private static final CatzRobotTracker instance = new CatzRobotTracker();

    private final CatzDrivetrain DRIVE_TRAIN = CatzDrivetrain.getInstance();

    private Pose2d currentPos;
    private SwerveDriveOdometry swerveOdometry;

    private CatzRobotTracker() {
        super(20);


        //Initializing robot position
        swerveOdometry = new SwerveDriveOdometry(
            CatzAutonomous.getSwervedrivekinematics(), Rotation2d.fromDegrees(DRIVE_TRAIN.getGyroAngle()),
            new SwerveModulePosition[] {
                getModulePosition(DRIVE_TRAIN.LT_FRNT_MODULE),
                getModulePosition(DRIVE_TRAIN.LT_BACK_MODULE),
                getModulePosition(DRIVE_TRAIN.RT_FRNT_MODULE),
                getModulePosition(DRIVE_TRAIN.RT_BACK_MODULE)
            }, new Pose2d());
    }


    public static CatzRobotTracker getInstance(){
        return instance;
    }

    //Method called in the CommandTranslator
    public void resetPosition(Pose2d pose){
        swerveOdometry.resetPosition(Rotation2d.fromDegrees(DRIVE_TRAIN.getGyroAngle()), new SwerveModulePosition[] {
            getModulePosition(DRIVE_TRAIN.LT_FRNT_MODULE),
            getModulePosition(DRIVE_TRAIN.LT_BACK_MODULE),
            getModulePosition(DRIVE_TRAIN.RT_FRNT_MODULE),
            getModulePosition(DRIVE_TRAIN.RT_BACK_MODULE)
        }, pose);
    }

    public Pose2d getCurrentPos() {
        return currentPos;
    }

    private void updateRobotPosition(){
        swerveOdometry.update(Rotation2d.fromDegrees(DRIVE_TRAIN.getGyroAngle()), new SwerveModulePosition[] {
            getModulePosition(DRIVE_TRAIN.LT_FRNT_MODULE),
            getModulePosition(DRIVE_TRAIN.LT_BACK_MODULE),
            getModulePosition(DRIVE_TRAIN.RT_FRNT_MODULE),
            getModulePosition(DRIVE_TRAIN.RT_BACK_MODULE)
        });
        
        currentPos = swerveOdometry.getPoseMeters();
    }

    @Override
    public void update() {
        updateRobotPosition();
    }
    
    private SwerveModulePosition getModulePosition(CatzSwerveModule module){
        double distanceMoved = (double) Units.inchesToMeters(module.getDriveDistance()); 

        return new SwerveModulePosition(
            distanceMoved, new Rotation2d(Math.toRadians(module.getAngle()))
        );
    }
    @Override
    public void close() throws Exception {}
    @Override
    public void selfTest() {}
}
