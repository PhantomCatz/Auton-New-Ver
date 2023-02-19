package frc.Utils;

public class CatzMathUtils {
    
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public static double calcJoystickAngle(double xJoy, double yJoy)
    {
        double angle = Math.atan(Math.abs(xJoy) / Math.abs(yJoy));
        angle *= (180 / Math.PI);

        if(yJoy <= 0)   //joystick pointed up
        {
            if(xJoy < 0)    //joystick pointed left
            {
                //no change
            }
            if(xJoy >= 0)   //joystick pointed right
            {
                angle = -angle;
            }
        }
        else    //joystick pointed down
        {
            if(xJoy < 0)    //joystick pointed left
            {
                angle = 180 - angle;
            }
            if(xJoy >= 0)   //joystick pointed right
            {
                angle = -180 + angle;
            }
        }
        return angle;
    }

    public static double calcJoystickPower(double xJoy, double yJoy)
    {
      return (Math.sqrt(Math.pow(xJoy, 2) + Math.pow(yJoy, 2)));
    }

}
