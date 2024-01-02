/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492.drivebases;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Scanner;

import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcGyro;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcAHRSGyro;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcCANSparkMax;
import TrcFrcLib.frclib.FrcCANTalon;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team492.FrcAuto;
import team492.Robot;
import team492.RobotParams;

/**
 * This class is intended to be extended by subclasses implementing different robot drive bases.
 */
public class RobotDrive
{
    private static final String moduleName = RobotDrive.class.getSimpleName();
    public static final int INDEX_LEFT_FRONT = 0;
    public static final int INDEX_RIGHT_FRONT = 1;
    public static final int INDEX_LEFT_BACK = 2;
    public static final int INDEX_RIGHT_BACK = 3;
    private static final String FIELD_ZERO_HEADING_FILE = "FieldZeroHeading.txt";

    public enum MotorType
    {
        CanFalcon,
        CanTalon,
        CanSparkMax
    }   //enum MotorType

    /**
     * This enum specifies different drive modes.
     */
    public enum DriveMode
    {
        TankMode,
        HolonomicMode,
        ArcadeMode
    }   //enum DriveMode

    //
    // Global objects.
    //
    protected final Robot robot;

    //
    // Sensors.
    //
    public final TrcGyro gyro;
    //
    // Subclass needs to initialize the following variables.
    //
    // Drive motors.
    public TrcMotor[] driveMotors;
    // Drive Base.
    public TrcDriveBase driveBase;
    // Drive Controllers.
    public TrcPidDrive pidDrive;
    public TrcPurePursuitDrive purePursuitDrive;
    //
    // PID Coefficients.
    //
    public TrcPidController.PidCoefficients xPosPidCoeff, yPosPidCoeff, turnPidCoeff, velPidCoeff;
    public TrcPidController.PidCoefficients gyroPitchPidCoeff;      // for anti-tipping.
    //
    // Odometry.
    //
    private TrcPose2D endOfAutoRobotPose = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public RobotDrive(Robot robot)
    {
        this.robot = robot;
        gyro = RobotParams.Preferences.useNavX ? new FrcAHRSGyro("NavX", SPI.Port.kMXP) : null;
    }   //RobotDrive

    /**
     * This method is called to prepare the robot base before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    public void startMode(RunMode runMode, RunMode prevMode)
    {
        if (runMode != RunMode.DISABLED_MODE)
        {
            driveBase.setOdometryEnabled(true, true);
            // Disable ramp rate control in autonomous.
            double rampTime = runMode == RunMode.AUTO_MODE? 0.0: RobotParams.DRIVE_RAMP_RATE;
            for (int i = 0; i < driveMotors.length; i++)
            {
                driveMotors[i].setOpenLoopRampRate(rampTime);
            }

            if (runMode != RunMode.AUTO_MODE)
            {
                if (runMode == RunMode.TELEOP_MODE && endOfAutoRobotPose != null)
                {
                    driveBase.setFieldPosition(endOfAutoRobotPose);
                    endOfAutoRobotPose = null;
                }

                if (RobotParams.Preferences.useGyroAssist)
                {
                    driveBase.setGyroAssistEnabled(pidDrive.getTurnPidCtrl());
                }
            }
        }
    }   //startMode

    /**
     * This method is called to prepare the robot base right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    public void stopMode(RunMode runMode, RunMode nextMode)
    {
        if (runMode != RunMode.DISABLED_MODE)
        {
            cancel();

            if (runMode == RunMode.AUTO_MODE)
            {
                endOfAutoRobotPose = driveBase.getFieldPosition();
            }
            driveBase.setOdometryEnabled(false);
        }
    }   //stopMode

    /**
     * This method cancels any PIDDrive operation still in progress.
     *
     * @param owner specifies the owner that requested the cancel.
     */
    public void cancel(String owner)
    {
        if (pidDrive != null && pidDrive.isActive())
        {
            pidDrive.cancel(owner);
        }

        if (purePursuitDrive != null && purePursuitDrive.isActive())
        {
            purePursuitDrive.cancel(owner);
        }

        driveBase.stop(owner);
    }   //cancel

    /**
     * This method cancels any PIDDrive operation still in progress.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method create an array of motors and configure them (can be drive motor or steer motor for Swerve Drive).
     *
     * @param motorType specifies the motor type (CAN_FALCON, CAN_TALON or CAN_SPARKMAX).
     * @param brushless specifies true if motor is brushless, false if brushed (only applicable for SparkMax).
     * @param names specifies an array of names for each motor.
     * @param motorCanIds specifies an array of CAN IDs for each motor.
     * @param inverted specifies an array of boolean indicating if the motor needs to be inverted.
     * @return created array of motors.
     */
    protected TrcMotor[] createMotors(
        MotorType motorType, boolean brushless, String[] names, int[] motorCanIds, boolean[] inverted)
    {
        TrcMotor[] motors = new TrcMotor[names.length];

        for (int i = 0; i < names.length; i++)
        {
            switch (motorType)
            {
                case CanFalcon:
                    motors[i] = new FrcCANFalcon(names[i], motorCanIds[i]);
                    break;

                case CanTalon:
                    motors[i] = new FrcCANTalon(names[i], motorCanIds[i]);
                    break;

                case CanSparkMax:
                    motors[i] = new FrcCANSparkMax(names[i], motorCanIds[i], brushless);
                    break;
            }
            motors[i].resetFactoryDefault();
            motors[i].setVoltageCompensationEnabled(RobotParams.BATTERY_NOMINAL_VOLTAGE);
            motors[i].setBrakeModeEnabled(true);
            motors[i].setMotorInverted(inverted[i]);
        }

        return motors;
    }   //createMotors

    /**
     * This method reads various joystick/gamepad control values and returns the drive powers for all three degrees
     * of robot movement.
     *
     * @param driveMode specifies the drive mode which determines the control mappings.
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     * @param drivePowerScale specifies the scaling factor for drive power.
     * @param turnPowerScale specifies the scaling factor for turn power.
     * @return an array of 3 values for x, y and rotation power.
     */
    public double[] getDriveInputs(
        DriveMode driveMode, boolean doExp, double drivePowerScale, double turnPowerScale)
    {
        double x = 0.0, y = 0.0, rot = 0.0;

        switch (driveMode)
        {
            case HolonomicMode:
                if (RobotParams.Preferences.useDriverXboxController)
                {
                    x = robot.driverController.getRightXWithDeadband(doExp);
                    y = robot.driverController.getLeftYWithDeadband(doExp);
                    rot = robot.driverController.getTriggerWithDeadband(doExp);
                }
                else
                {
                    x = robot.rightDriveStick.getXWithDeadband(doExp);
                    y = robot.leftDriveStick.getYWithDeadband(doExp);
                    rot = robot.leftDriveStick.getTwistWithDeadband(doExp);
                }
                robot.globalTracer.traceDebug(moduleName, driveMode + ":x=" + x + ",y=" + y + ",rot=" + rot);
                break;

            case ArcadeMode:
                if (RobotParams.Preferences.useDriverXboxController)
                {
                    x = robot.driverController.getLeftXWithDeadband(doExp);
                    y = robot.driverController.getLeftYWithDeadband(doExp);
                    rot = robot.driverController.getRightXWithDeadband(doExp);
                }
                else
                {
                    x = robot.leftDriveStick.getXWithDeadband(doExp);
                    y = robot.leftDriveStick.getYWithDeadband(doExp);
                    if (RobotParams.Preferences.doOneStickDrive)
                    {
                        rot = robot.leftDriveStick.getTwistWithDeadband(doExp);
                    }
                    else
                    {
                        rot = robot.rightDriveStick.getXWithDeadband(doExp);
                    }
                }
                robot.globalTracer.traceDebug(moduleName, driveMode + ":x=" + x + ",y=" + y + ",rot=" + rot);
                break;

            case TankMode:
                double leftPower, rightPower;
                if (RobotParams.Preferences.useDriverXboxController)
                {
                    leftPower = robot.driverController.getLeftYWithDeadband(doExp);
                    rightPower = robot.driverController.getRightYWithDeadband(doExp);
                }
                else
                {
                    leftPower = robot.leftDriveStick.getYWithDeadband(false);
                    rightPower = robot.rightDriveStick.getYWithDeadband(false);
                }
                x = 0.0;
                y = (leftPower + rightPower)/2.0;
                rot = (leftPower - rightPower)/2.0;
                robot.globalTracer.traceDebug(moduleName, driveMode + ":left=" + leftPower + ",right=" + rightPower);
                break;
        }

        double mag = TrcUtil.magnitude(x, y);
        if (mag > 1.0)
        {
            x /= mag;
            y /= mag;
        }
        x *= drivePowerScale;
        y *= drivePowerScale;
        rot *= turnPowerScale;

        return new double[] { x, y, rot };
    }   //getDriveInput

    /**
     * This method reads various joystick/gamepad control values and returns the drive powers for all three degrees
     * of robot movement.
     *
     * @param driveMode specifies the drive mode which determines the control mappings.
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *              raised exponentially, it gives you more precise control on the low end values.
     * @return an array of 3 values for x, y and rotation power.
     */
    public double[] getDriveInputs(DriveMode driveMode, boolean doExp)
    {
        return getDriveInputs(driveMode, doExp, 1.0, 1.0);
    }   //getDriveInputs

    /**
     * This method saves the compass heading value when the robot is facing field zero.
     */
    public void saveFieldZeroCompassHeading()
    {
        try (PrintStream out = new PrintStream(
                new FileOutputStream(RobotParams.TEAM_FOLDER_PATH + "/" + FIELD_ZERO_HEADING_FILE)))
        {
            double fieldZeroHeading = ((FrcAHRSGyro) gyro).ahrs.getCompassHeading();

            out.println(fieldZeroHeading);
            out.close();
            robot.globalTracer.traceInfo(moduleName, "FieldZeroCompassHeading=" + fieldZeroHeading);
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }   //saveFieldZeroCompassHeading

    /**
     * This method retrieves the field zero compass heading from the calibration data file.
     *
     * @return calibration data of field zero compass heading.
     */
    private Double getFieldZeroCompassHeading()
    {
        try (Scanner in = new Scanner(new FileReader(RobotParams.TEAM_FOLDER_PATH + "/" + FIELD_ZERO_HEADING_FILE)))
        {
            return in.nextDouble();
        }
        catch (Exception e)
        {
            robot.globalTracer.traceWarn(moduleName, "FieldZeroHeading file not found.");
            return null;
        }
    }   //getFieldZeroHeading

    /**
     * This method sets the robot's absolute field position. This is typically called at the beginning of a match for
     * robot localization. The provided pose should be the robot's starting position. If null, it will try to get the
     * robot start pose from the auto choices on the dashboard. Optionally, the caller can set useCompassHeading to
     * true for using compass heading to determine the true robot heading. This only works if the robot has been
     * calibrated on the competition field for its field zero position.
     * Note: if reading the field zero calibration file failed, it will behave as if useCompassHeading is false.
     *
     * @param pose speicifies the robot's starting position on the field.
     * @param useCompassHeading specifies true to use compass to determine the robot's true heading, false otherwise.
     */
    public void setFieldPosition(TrcPose2D pose, boolean useCompassHeading)
    {
        TrcPose2D robotPose;

        if (pose == null)
        {
            int startPos = FrcAuto.autoChoices.getStartPos();
            robotPose = FrcAuto.autoChoices.getAlliance() == Alliance.Blue?
                RobotParams.startPos[0][startPos]: RobotParams.startPos[1][startPos];
        }
        else
        {
            robotPose = pose.clone();
        }

        if (useCompassHeading)
        {
            Double fieldZero = getFieldZeroCompassHeading();

            if (fieldZero != null)
            {
                robotPose.angle = ((FrcAHRSGyro) gyro).ahrs.getCompassHeading() - fieldZero;
            }
        }

        driveBase.setFieldPosition(robotPose);
    }   //setFieldPosition

    /**
     * This method sets the robot's absolute field position. This is typically called at the beginning of a match for
     * robot localization. The provided pose should be the robot's starting position. If null, it will try to get the
     * robot start pose from the auto choices on the dashboard. Optionally, the caller can set  useCompassHeading to
     * true for using compass heading to determine the true robot heading. This only works if the robot has been
     * calibrated on the competition field for its field zero position.
     * Note: if reading the field zero calibration file failed, it will behave as if useCompassHeading is false.
     *
     * @param useCompassHeading specifies true to use compass to determine the robot's true heading, false otherwise.
     */
    public void setFieldPosition(boolean useCompassHeading)
    {
        setFieldPosition(null, useCompassHeading);
    }   //setFieldPosition

    /**
     * This method returns the gyro pitch.
     *
     * @return gyro pitch.
     */
    public double getGyroPitch()
    {
        return gyro.getXHeading().value;
    }   //getGyroPitch

    /**
     * This method returns the gyro roll.
     *
     * @return gyro roll.
     */
    public double getGyroRoll()
    {
        return gyro.getYHeading().value;
    }   //getGyroRoll

    /**
     * This method returns an adjusted absolute position by the robot's alliance.
     *
     * @param alliance specifies the robot alliance.
     * @param pose specifies the absolute position for the blue alliance.
     * @return returns unchanged pos if blue alliance, adjusted to the opposite side if red alliance.
     */
    public TrcPose2D adjustPoseByAlliance(Alliance alliance, TrcPose2D pose)
    {
        if (alliance == Alliance.Red)
        {
            // no change on x, change y to the opposite side of the field.
            pose.y = RobotParams.FIELD_LENGTH - pose.y;
            pose.angle = (pose.angle + 180.0) % 360.0;
        }

        return pose;
    }   //adjustPoseByAlliance

}   //class RobotDrive
