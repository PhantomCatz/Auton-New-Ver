package frc.Autons;


import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.CatzDrivetrain;
import frc.Utils.CatzMathUtils;
import frc.robot.CatzConstants;


public class CatzAutonomous extends ThreadRunner{
    //Singleton Instance
    private static final CatzAutonomous autonomousInstance = new CatzAutonomous();


    private final CatzDrivetrain driveTrain = CatzDrivetrain.getDrivetrainInstance();

    private final HolonomicDriveController holonomicDriveController;

    private static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        CatzConstants.SWERVE_LEFT_FRONT_LOCATION,
        CatzConstants.SWERVE_LEFT_BACK_LOCATION,
        CatzConstants.SWERVE_RIGHT_FRONT_LOCATION,
        CatzConstants.SWERVE_RIGHT_BACK_LOCATION
    );


    private Trajectory currentTrajectory;
    private Rotation2d targetRotation;

    private double autoStartTime;

    private boolean isDone = false;

    private static int TRAJECTORY_FOLLOWER_THREAD_PERIOD_MS = 20;

    private CatzAutonomous(){
        super(TRAJECTORY_FOLLOWER_THREAD_PERIOD_MS);

        ProfiledPIDController autoTurnPIDController
            = new ProfiledPIDController(8, 0, 0.01, new TrapezoidProfile.Constraints(4, 4));
            autoTurnPIDController.enableContinuousInput(-Math.PI, Math.PI);
            autoTurnPIDController.setTolerance(Math.toRadians(10));

        holonomicDriveController = new HolonomicDriveController(
            new PIDController(0.2, 0,0),
            new PIDController(0.2, 0, 0),
            autoTurnPIDController
        );

        
    }

    //Methods used in the CommandTranslator starts here. --------------------------------
    public void setAutoPath(Trajectory trajectory) {
        this.currentTrajectory = trajectory;
        autoStartTime = Timer.getFPGATimestamp();
    }

    public void setAutoRotation(Rotation2d rotation) {
        this.targetRotation = rotation;
    }

    public void stopMovement() {
        isDone = true;
    }

    synchronized public boolean isFinished() {
        return isDone;
    }

    public double getAutoElapsedTime() {
        return Timer.getFPGATimestamp() - autoStartTime;
    }
    //Methods used in the CommandTranslator ends here. --------------------------------


    private void runAuto(){
        Pose2d currentPos = CatzRobotTracker.getRobotTrackerInstance().getCurrentPose();

        Trajectory.State goal = currentTrajectory.sample(Timer.getFPGATimestamp() - autoStartTime);
        ChassisSpeeds adjustedSpeed = holonomicDriveController.calculate(currentPos, goal, targetRotation); 

        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(adjustedSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, CatzConstants.MAX_AUTON_SPEED_METERS_PER_SECOND);

        setSwerveModule(swerveModuleStates);

        //Checking if the robot has reached the destination and if the current time has reached the predicted finishing time.
        if((holonomicDriveController.atReference()) && ((Timer.getFPGATimestamp() - autoStartTime) >= currentTrajectory.getTotalTimeSeconds())){
            isDone = true;
        }

        
    }

    private void setSwerveModule(SwerveModuleState[] moduleStates){
        for(int module = 0; module < 4; module++){
            SwerveModuleState moduleState = moduleStates[module];

            double targetAngle = moduleState.angle.getDegrees() % 360.0;
            double speed = CatzMathUtils.clamp(-CatzConstants.MAX_AUTON_SPEED_METERS_PER_SECOND,CatzConstants.MAX_AUTON_SPEED_METERS_PER_SECOND,moduleState.speedMetersPerSecond) / CatzConstants.MAX_AUTON_SPEED_METERS_PER_SECOND;
            
            driveTrain.setOneModuleDrivePower(module, speed);
            driveTrain.setOneModuleWheelRotation(module, targetAngle);
        }
    }


    public static CatzAutonomous getAutonomousinstance(){
        return autonomousInstance;
    }

    public static SwerveDriveKinematics getSwervedrivekinematics(){
        return swerveDriveKinematics;
    }

    @Override
    public void update(){
        if(isDone){
            driveTrain.setDrivePower(0);
        }
        else{
            runAuto();
        }
    }

    public void updateShuffleboard(){
        SmartDashboard.putBoolean("Is Done", isDone);
    }
}
