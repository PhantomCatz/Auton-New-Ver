package frc.Mechanisms;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;

public class CatzDrivetrain 
{
    //Singleton Instance
    private static final CatzDrivetrain driveTrainInstance = new CatzDrivetrain();

    private enum FrontSide{
        FRONT("Front"),
        LEFT("Left"),
        RIGHT("Right"),
        BACK("Back");

        private final String side;

        private FrontSide(String side){
            this.side = side;
        }
    }

    public final CatzSwerveModule RT_FRNT_MODULE;
    public final CatzSwerveModule RT_BACK_MODULE;
    public final CatzSwerveModule LT_FRNT_MODULE;
    public final CatzSwerveModule LT_BACK_MODULE;

    public final CatzSwerveModule[] swerveModules = new CatzSwerveModule[4];

    private AHRS navX;

    private final int LT_FRNT_DRIVE_ID = 1;
    private final int LT_BACK_DRIVE_ID = 3;
    private final int RT_BACK_DRIVE_ID = 5;
    private final int RT_FRNT_DRIVE_ID = 7;
    
    private final int LT_FRNT_STEER_ID = 2;
    private final int LT_BACK_STEER_ID = 4;
    private final int RT_BACK_STEER_ID = 6;
    private final int RT_FRNT_STEER_ID = 8;

    private final int LT_FRNT_ENC_PORT = 9;
    private final int LT_BACK_ENC_PORT = 6;
    private final int RT_BACK_ENC_PORT = 7;
    private final int RT_FRNT_ENC_PORT = 8;

    private double LT_FRNT_OFFSET = 0.0055;
    private double LT_BACK_OFFSET = 0.3592;
    private double RT_BACK_OFFSET = 0.0668;
    private double RT_FRNT_OFFSET = 0.6513;

    private CatzLog data;
    private FrontSide front = FrontSide.FRONT;

    public CatzDrivetrain()
    {
        LT_FRNT_MODULE = new CatzSwerveModule(LT_FRNT_DRIVE_ID, LT_FRNT_STEER_ID, LT_FRNT_ENC_PORT, LT_FRNT_OFFSET);
        LT_BACK_MODULE = new CatzSwerveModule(LT_BACK_DRIVE_ID, LT_BACK_STEER_ID, LT_BACK_ENC_PORT, LT_BACK_OFFSET);
        RT_FRNT_MODULE = new CatzSwerveModule(RT_FRNT_DRIVE_ID, RT_FRNT_STEER_ID, RT_FRNT_ENC_PORT, RT_FRNT_OFFSET);
        RT_BACK_MODULE = new CatzSwerveModule(RT_BACK_DRIVE_ID, RT_BACK_STEER_ID, RT_BACK_ENC_PORT, RT_BACK_OFFSET);

        swerveModules[0] = LT_FRNT_MODULE;
        swerveModules[2] = LT_BACK_MODULE;
        swerveModules[1] = RT_FRNT_MODULE;
        swerveModules[3] = RT_BACK_MODULE;

        navX = new AHRS();
        navX.reset();
        navX.setAngleAdjustment(74.0);
 
        LT_FRNT_MODULE.resetMagEnc();
        LT_BACK_MODULE.resetMagEnc();
        RT_FRNT_MODULE.resetMagEnc();
        RT_BACK_MODULE.resetMagEnc();
    }

    public void initializeOffsets()
    {
        navX.setAngleAdjustment(-navX.getYaw());

        LT_FRNT_MODULE.initializeOffset();
        LT_BACK_MODULE.initializeOffset();
        RT_BACK_MODULE.initializeOffset();
        RT_FRNT_MODULE.initializeOffset();
    }

    public void drive(double joystickAngle, double joystickPower)
    {
        LT_FRNT_MODULE.setWheelAngle(joystickAngle, getGyroAngle());
        LT_BACK_MODULE.setWheelAngle(joystickAngle, getGyroAngle());
        RT_FRNT_MODULE.setWheelAngle(joystickAngle, getGyroAngle());
        RT_BACK_MODULE.setWheelAngle(joystickAngle, getGyroAngle());

        setDrivePower(joystickPower);
    }

    public void setOneModuleWheelRotation(int i, double angle){
        swerveModules[i].setWheelAngle(i, angle);
    }

    public void setOneModuleDrivePower(int i, double pwr){
        swerveModules[i].setDrivePower(pwr);
    }

    public double getOneWheelAngle(int i){
        return swerveModules[i].getAngle();
    }

    public void rotateInPlace(double pwr)
    {
        LT_FRNT_MODULE.setWheelAngle(-45.0, 0.0);
        LT_BACK_MODULE.setWheelAngle(45.0, 0.0);
        RT_FRNT_MODULE.setWheelAngle(-135.0, 0.0);
        RT_BACK_MODULE.setWheelAngle(135.0, 0.0);

        pwr *= 0.6;

        LT_FRNT_MODULE.setDrivePower(pwr);
        LT_BACK_MODULE.setDrivePower(pwr);
        RT_FRNT_MODULE.setDrivePower(pwr);
        RT_BACK_MODULE.setDrivePower(pwr);
    }

    public void translateTurn(double direction, double translatePower, double turnPower)
    {
        //how far wheels turn determined by how far joystick is pushed (max of 45 degrees)
        double turnAngle = turnPower * -45.0;

        double gyroAngle = getGyroAngle();

        // if directed towards front of robot
        if(Math.abs(closestAngle(direction, 0.0 - gyroAngle)) <= 45.0)
        {
            LT_FRNT_MODULE.setWheelAngle(direction + turnAngle, gyroAngle);
            RT_FRNT_MODULE.setWheelAngle(direction + turnAngle, gyroAngle);

            LT_BACK_MODULE.setWheelAngle(direction - turnAngle, gyroAngle);
            RT_BACK_MODULE.setWheelAngle(direction - turnAngle, gyroAngle);

            front = FrontSide.FRONT;
        }
        // if directed towards left of robot
        else if(Math.abs(closestAngle(direction, 90.0 - gyroAngle)) < 45.0)
        {
            LT_FRNT_MODULE.setWheelAngle(direction + turnAngle, gyroAngle);
            LT_BACK_MODULE.setWheelAngle(direction + turnAngle, gyroAngle);

            RT_FRNT_MODULE.setWheelAngle(direction - turnAngle, gyroAngle);
            RT_BACK_MODULE.setWheelAngle(direction - turnAngle, gyroAngle);

            front = FrontSide.LEFT;
        }
        // if directed towards back of robot
        else if(Math.abs(closestAngle(direction, 180.0 - gyroAngle)) <= 45.0)
        {
            LT_BACK_MODULE.setWheelAngle(direction + turnAngle, gyroAngle);
            RT_BACK_MODULE.setWheelAngle(direction + turnAngle, gyroAngle);

            LT_FRNT_MODULE.setWheelAngle(direction - turnAngle, gyroAngle);
            RT_FRNT_MODULE.setWheelAngle(direction - turnAngle, gyroAngle);

            front = FrontSide.BACK;

        }
        // if directed towards right of robot
        else if(Math.abs(closestAngle(direction, -90.0 - gyroAngle)) < 45.0)
        {
            RT_FRNT_MODULE.setWheelAngle(direction + turnAngle, gyroAngle);
            RT_BACK_MODULE.setWheelAngle(direction + turnAngle, gyroAngle);

            LT_FRNT_MODULE.setWheelAngle(direction - turnAngle, gyroAngle);
            LT_BACK_MODULE.setWheelAngle(direction - turnAngle, gyroAngle);

            front = FrontSide.BACK;
        }


        LT_FRNT_MODULE.setDrivePower(translatePower);
        LT_BACK_MODULE.setDrivePower(translatePower);
        RT_FRNT_MODULE.setDrivePower(translatePower);
        RT_BACK_MODULE.setDrivePower(translatePower);
    }

    public double closestAngle(double a, double b)
    {
        // get direction
        double dir = b % 360.0 - a % 360.0;

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0)
        {
                dir = -(Math.signum(dir) * 360.0) + dir;
                //closest angle shouldn't be more than 180 degrees. If it is, use other direction
                if(dir > 180.0)
                {
                    dir -= 360;
                }
        }

        return dir;
    }

    public double getGyroAngle()
    {
        return navX.getAngle();
    }

    public void setSteerPower(double pwr)
    {
        LT_FRNT_MODULE.setSteerPower(pwr);
        LT_BACK_MODULE.setSteerPower(pwr);
        RT_FRNT_MODULE.setSteerPower(pwr);
        RT_BACK_MODULE.setSteerPower(pwr);
    }

    public void setDrivePower(double pwr)
    {
        LT_FRNT_MODULE.setDrivePower(pwr);
        LT_BACK_MODULE.setDrivePower(pwr);
        RT_FRNT_MODULE.setDrivePower(pwr);
        RT_BACK_MODULE.setDrivePower(pwr);
    }

    public void setBrakeMode()
    {
        LT_FRNT_MODULE.setBrakeMode();
        LT_BACK_MODULE.setBrakeMode();
        RT_FRNT_MODULE.setBrakeMode();
        RT_BACK_MODULE.setBrakeMode();
    }
    public void setCoastMode()
    {
        LT_FRNT_MODULE.setCoastMode();
        LT_BACK_MODULE.setCoastMode();
        RT_FRNT_MODULE.setCoastMode();
        RT_BACK_MODULE.setCoastMode();
    }

    public void updateShuffleboard()
    {
        SmartDashboard.putNumber("NavX", navX.getAngle());

        LT_FRNT_MODULE.updateShuffleboard();
        LT_BACK_MODULE.updateShuffleboard();
        RT_FRNT_MODULE.updateShuffleboard();
        RT_BACK_MODULE.updateShuffleboard();
    }

    public static CatzDrivetrain getDrivetrainInstance(){
        return driveTrainInstance;
    }

    public void dataCollection(){
        data = new CatzLog(
            Robot.getCurrentTime(),
            LT_FRNT_MODULE.getDrvPower(), LT_FRNT_MODULE.getAngle(),
            LT_BACK_MODULE.getDrvPower(), LT_BACK_MODULE.getAngle(),
            RT_FRNT_MODULE.getDrvPower(), RT_FRNT_MODULE.getAngle(),
            RT_BACK_MODULE.getDrvPower(), RT_BACK_MODULE.getAngle(),
            0,0,0,0,0,0,0
            );  
        Robot.getDataCollection().logData.add(data);
    }

}
