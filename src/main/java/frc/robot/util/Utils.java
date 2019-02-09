package frc.robot.util;

import frc.robot.Constants;

public class Utils {

    /**
     * Converts CANTalon's native units to revolutions
     * @param encoderNativeUnits encoder native units
     * @return encoder revolutions
     */
    public static double encUnitsToRevolutions(double encoderNativeUnits){
        //CTRE Mag Encoders have 4096 native units per revolution
        return encoderNativeUnits / 4096;
    }

    /**
     * Converts CANTalon's native units to inches traveled by the robot
     * @param encoderNativeUnits encoder native units
     * @return Inches traveled by the robot
     */
    public static double encUnitsToInches(double encoderNativeUnits){
        //There are exactly {wheelCircumference} inches per revolution

        //So:
        //{distance} inches = {encoderPosRevolutions} revolutions * {wheelCircumference} inches / 1 revolutions
        //{distance} inches = {encoderPosRevolutions} * {wheelCircumference} inches
        return encUnitsToRevolutions(encoderNativeUnits)*Constants.kChassisWheelCircumferenceInch;
    }

    public static double inchesToRevolutions(double inches){
        //Use for going from inches traveled (or wanting to be traveled) to encoder units (most likely a target value)
        return inches/ Constants.kChassisWheelCircumferenceInch;
    }

    public static double revolutionsToEncUnits(double inches){
        //
        return 4096*inchesToRevolutions(inches);
    }

    /**
     * Converts CANTalon.getEncVelocity() to RPM
     * @param encoderVelocity in native units per 100ms
     * @return encoderVelocity in revolutions per minute
     */
    public static double encVelToRPM(double encoderVelocity){
        //Encoder Velocity is given in native units per 100ms (native units per 0.1 sec)
        //encUnitsToRevolutions converts from (native units / 100ms) to (revolutions / 100ms)
        //To convert from (revolutions / 100ms) to (revolutions / minute), multiply by 600
        return 600*encUnitsToRevolutions(encoderVelocity);
    }

    /**
     * Converts CANTalon.getEncVelocity() to Inches/sec
     * @param encoderVelocity in native units per 100ms
     * @return encoderVelocity in inches per second
     */
    public static double encVelToInchesPerSecond(double encoderVelocity){
        //Encoder Velocity is given in native units per 100ms (native units per 0.1 sec)
        //encUnitsToInches converts from (native units / 100ms) to (inches / 100ms)
        //To convert from (inches / 100ms) to (inches / second), multiply by 10
        return 10*encUnitsToInches(encoderVelocity);
    }
}