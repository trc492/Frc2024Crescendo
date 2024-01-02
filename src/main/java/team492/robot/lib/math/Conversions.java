package team492.robot.lib.math;

public class Conversions {

    /**
     * @param encoderRot Encoder Position: (in Rotations)
     * @param gearRatio Gear Ratio between Encoder and Mechanism
     * @return Mechanism Position: (in Degrees)
     */
    public static double encoderToDegrees(double encoderRot, double gearRatio) {
        double mechRot = encoderRot / gearRatio;
        double mechDeg = mechRot * 360.0;
        return mechDeg;
    }

    /**
     * @param mechDeg Mechanism Position: (in Degrees)
     * @param gearRatio Gear Ratio between Encoder and Mechanism
     * @return Encoder Rotation: (in Rotations)
     */
    public static double degreesToEncoder(double mechDeg, double gearRatio) {
        double mechRot = mechDeg / 360.0;
        double encoderRot = mechRot * gearRatio;
        return encoderRot;
    }

    /**
     * @param motorRot Motor Position: (in Rotations)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Mechanism Position: (in Degrees)
     */
    public static double motorRotToDegrees(double motorRot, double gearRatio) {
        double motorDeg = motorRot * 360.0;
        double mechDeg = motorDeg / gearRatio;
        return mechDeg;
    }

    /**
     * @param encoderCounts Motor Position: (in encoder counts)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @param motorCPR Motor encoder count per revolution
     * @return Mechanism Position: (in Degrees)
     */
    public static double motorCountsToDegrees(double encoderCounts, double gearRatio, double motorCPR)
    {
        return motorRotToDegrees(encoderCounts / motorCPR, gearRatio);
    }

    /**
     * @param degrees Mechanism Position: (in Degrees)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Motor Rotation: (in Rotations)
     */
    public static double degreesToMotorRot(double mechDeg, double gearRatio) {
        double motorDeg = mechDeg * gearRatio;
        double motorRotations = motorDeg / 360.0;
        return motorRotations;
    }    

    /**
     * @param degrees Mechanism Position: (in Degrees)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @param motorCPR Motor encoder count per revolution
     * @return Motor Position: (in encoder counts)
     */
    public static double degreesToMotorCounts(double mechDeg, double gearRatio, double motorCPR) {
        return degreesToMotorRot(mechDeg, gearRatio) * motorCPR;
    }    

    /**
     * @param motorRPS Motor Velocity: (in Rotations per Second)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Mechanism Velocity: (in Rotations per Minute)
     */
    public static double motorRPSToRPM(double motorRPS, double gearRatio) {
        double motorRPM = motorRPS * 60.0;
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param motorVel Motor Velocity: (in encoder count per 100 msec)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @param motorCPR Motor encoder count per revolution
     * @return Mechanism Velocity: (in Rotations per Minute)
     */
    public static double motorVelToRPM(double motorVel, double gearRatio, double motorCPR) {
        return motorRPSToRPM(motorVel * 10.0 / motorCPR, gearRatio);
    }

    /**
     * @param mechRPM Mechanism Velocity: (in Rotations per Minute)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Motor Velocity: (in Rotations per Second)
     */
    public static double RPMToMotorRPS(double mechRPM, double gearRatio) {
        double motorRPM = mechRPM * gearRatio;
        double motorRPS = motorRPM / 60.0;
        return motorRPS;
    }

    /**
     * @param mechRPM Mechanism Velocity: (in Rotations per Minute)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @param motorCPR Motor encoder count per revolution
     * @return Motor Velocity: (in encoder counts per 100 msec)
     */
    public static double RPMToMotorVel(double mechRPM, double gearRatio, double motorCPR) {
        return RPMToMotorRPS(mechRPM, gearRatio) * motorCPR / 10.0;
    }

    /**
     * @param motorRPS Motor Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double motorRPSToMPS(double motorRPS, double circumference, double gearRatio){
        double wheelRPS = motorRPS / gearRatio;
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param motorVel Motor Velocity: (in encoder counts per 100 msec)
     * @param circumference Wheel Circumference: (in Meters)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @param motorCPR Motor encoder count per revolution
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double motorVelToMPS(double motorVel, double circumference, double gearRatio, double motorCPR){
        return motorRPSToMPS(motorVel * 10.0 / motorCPR, circumference, gearRatio);
    }

    /**
     * @param wheelMPS Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @param gearRatio Gear Ratio between Motor and Wheel
     * @return Motor Velocity: (in Rotations per Second)
     */
    public static double MPSToMotorRPS(double wheelMPS, double circumference, double gearRatio){
        double wheelRPS = wheelMPS / circumference;
        double motorRPS = wheelRPS * gearRatio;
        return motorRPS;
    }

    /**
     * @param wheelMPS Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @param gearRatio Gear Ratio between Motor and Wheel
     * @param motorCPR Motor encoder count per revolution
     * @return Motor Velocity: (in encoder counts per 100 msec)
     */
    public static double MPSToMotorVel(double wheelMPS, double circumference, double gearRatio, double motorCPR){
        return MPSToMotorRPS(wheelMPS, circumference, gearRatio) * motorCPR / 10.0;
    }

    /**
     * @param motorRot Motor Position: (in Rotations)
     * @param circumference Wheel Circumference: (in Meters)
     * @param gearRatio Gear Ratio between Motor and Wheel
     * @return Wheel Distance: (in Meters)
     */
    public static double motorRotToMeters(double motorRot, double circumference, double gearRatio){
        double wheelRotations = motorRot / gearRatio;
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }

    /**
     * @param motorRot Motor Position: (in encoder counts)
     * @param circumference Wheel Circumference: (in Meters)
     * @param gearRatio Gear Ratio between Motor and Wheel
     * @param motorCPR Motor encoder count per revolution
     * @return Wheel Distance: (in Meters)
     */
    public static double motorCountsToMeters(double motorCounts, double circumference, double gearRatio, double motorCPR){
        return motorRotToMeters(motorCounts / motorCPR, circumference, gearRatio);
    }

    /**
     * @param wheelMeters Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @param gearRatio Gear Ratio between Motor and Wheel
     * @return Motor Position: (in Rotations)
     */
    public static double MetersToMotorRot(double wheelMeters, double circumference, double gearRatio){
        double wheelRotations = wheelMeters / circumference;
        double motorRotations = wheelRotations * gearRatio;
        return motorRotations;
    }

    /**
     * @param wheelMeters Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @param gearRatio Gear Ratio between Motor and Wheel
     * @param motorCPR Motor encoder count per revolution
     * @return Motor Position: (in Rotations)
     */
    public static double MetersToMotorCounts(double wheelMeters, double circumference, double gearRatio, double motorCPR){
        return MetersToMotorRot(wheelMeters, circumference, gearRatio) * motorCPR;
    }
}
