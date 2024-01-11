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

package team492;

import java.util.Locale;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcRobotBattery;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcAHRSGyro;
import TrcFrcLib.frclib.FrcDashboard;
import TrcFrcLib.frclib.FrcJoystick;
import TrcFrcLib.frclib.FrcLimeLightVision;
import TrcFrcLib.frclib.FrcMatchInfo;
import TrcFrcLib.frclib.FrcPdp;
import TrcFrcLib.frclib.FrcPhotonVision;
import TrcFrcLib.frclib.FrcRobotBase;
import TrcFrcLib.frclib.FrcRobotBattery;
import TrcFrcLib.frclib.FrcXboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team492.drivebases.RobotDrive;
import team492.drivebases.SwerveDrive;
import team492.robot.CTREConfigs;
import team492.robot.RobotContainer;
import team492.subsystems.LEDIndicator;
import team492.vision.LimeLightVision;
import team492.vision.OpenCvVision;
import team492.vision.PhotonVision;

/**
 * The Main class is configured to instantiate and automatically run this class,
 * and to call the functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main class to reflect the name change.
 */
public class Robot extends FrcRobotBase
{
    //
    // Global objects.
    //
    public final FrcDashboard dashboard = FrcDashboard.getInstance();
    public final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private double nextDashboardUpdateTime = TrcTimer.getModeElapsedTime();
    private boolean traceLogOpened = false;
    //
    // Hybrid mode objects.
    //
    public static CTREConfigs ctreConfigs;
    public RobotContainer m_robotContainer;
    public Command m_autonomousCommand;
    //
    // Inputs.
    //
    public FrcXboxController driverController;
    public FrcJoystick leftDriveStick, rightDriveStick;
    public FrcJoystick operatorStick;
    public FrcJoystick buttonPanel;
    public FrcJoystick switchPanel;
    //
    // Sensors.
    //
    public FrcPdp pdp;
    public TrcRobotBattery battery;
    public AnalogInput pressureSensor;
    //
    // Miscellaneous hardware.
    //
    public LEDIndicator ledIndicator;
    //
    // Vision subsystem.
    //
    public LimeLightVision limeLightVision;
    public PhotonVision photonVision;
    public OpenCvVision openCvVision;
    //
    // DriveBase subsystem.
    //
    public SwerveDrive robotDrive;
    //
    // Other subsystems.
    //

    /**
     * Constructor: Create an instance of the object.
     */
    public Robot()
    {
        super(RobotParams.ROBOT_NAME);
    }   //Robot

    /**
     * This method is called when the robot is first started up and should be used for any initialization code
     * including creation and initialization of all robot hardware and subsystems.
     *
     * To create new hardware or subsystem, follow the steps below:
     * 1. Create a public class variable for the new hardware/subsystem.
     * 2. Instantiate and initialize the new hardware/subsystem object in this method.
     * 3. Put code in updateDashboard to display status of the new hardware/subsystem if necessary.
     * 4. Put code in robotStartMode or robotStopMode to configure/reset hardware/subsystem if necessary.
     * 5. Put code in FrcTeleOp to operate the subsystem if necessary (i.e. slowPeriodic/xxxButtonEvent).
     * 6. Create a getter method for the new sensor only if necessary (e.g. sensor value needs translation).
     */
    @Override
    public void robotInit()
    {
        if (RobotParams.Preferences.hybridMode)
        {
            ctreConfigs = new CTREConfigs();
            // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
            // autonomous chooser on the dashboard.
            m_robotContainer = new RobotContainer();
        }
        //
        // Create and initialize global objects.
        //

        //
        // Create and initialize inputs.
        //
        // Give driver control to command-based if Hybrid Mode is ON.
        if (!RobotParams.Preferences.hybridMode)
        {
            if (RobotParams.Preferences.useDriverXboxController)
            {
                driverController = new FrcXboxController("DriverController", RobotParams.XBOX_DRIVER_CONTROLLER);
                driverController.setLeftYInverted(true);
                driverController.setRightYInverted(true);
            }
            else
            {
                leftDriveStick = new FrcJoystick("DriverLeftStick", RobotParams.JSPORT_DRIVER_LEFTSTICK);
                leftDriveStick.setYInverted(true);
                rightDriveStick = new FrcJoystick("DriverRightStick", RobotParams.JSPORT_DRIVER_RIGHTSTICK);
                rightDriveStick.setYInverted(true);
            }
        }

        operatorStick = new FrcJoystick("operatorStick", RobotParams.JSPORT_OPERATORSTICK);
        operatorStick.setYInverted(false);

        if (RobotParams.Preferences.useButtonPanels)
        {
            buttonPanel = new FrcJoystick("buttonPanel", RobotParams.JSPORT_BUTTON_PANEL);
            switchPanel = new FrcJoystick("switchPanel", RobotParams.JSPORT_SWITCH_PANEL);
        }
        //
        // Create and initialize sensors.
        //
        if (RobotParams.Preferences.usePdp)
        {
            pdp = new FrcPdp(RobotParams.CANID_PDP, ModuleType.kRev);
            pdp.setSwitchableChannel(false);
            battery = new FrcRobotBattery(pdp);
        }

        if (RobotParams.Preferences.usePressureSensor)
        {
            pressureSensor = new AnalogInput(RobotParams.AIN_PRESSURE_SENSOR);
        }
        //
        // Create and initialize miscellaneous hardware.
        //
        ledIndicator = new LEDIndicator();
        //
        // Create and initialize Vision subsystem.
        //
        if (RobotParams.Preferences.useVision)
        {
            if (RobotParams.Preferences.useLimeLightVision)
            {
                limeLightVision = new LimeLightVision("limelight");
            }

            if (RobotParams.Preferences.usePhotonVision)
            {
                photonVision = new PhotonVision("OV5647", ledIndicator);
            }

            if (RobotParams.Preferences.useOpenCvVision)
            {
                UsbCamera camera = CameraServer.startAutomaticCapture();
                camera.setResolution(RobotParams.CAMERA_IMAGE_WIDTH, RobotParams.CAMERA_IMAGE_HEIGHT);
                camera.setFPS(10);
                openCvVision = new OpenCvVision(
                    "OpenCvVision", 1, RobotParams.cameraRect, RobotParams.worldRect, CameraServer.getVideo(),
                    CameraServer.putVideo("UsbWebcam", RobotParams.CAMERA_IMAGE_WIDTH,
                    RobotParams.CAMERA_IMAGE_HEIGHT));
            }

            if (RobotParams.Preferences.useStreamCamera)
            {
                UsbCamera camera = CameraServer.startAutomaticCapture("DriverDisplay", 0);
                camera.setResolution(160, 120);
                camera.setFPS(10);
            }
        }
        //
        // Create and initialize RobotDrive subsystem.
        //
        if (!RobotParams.Preferences.hybridMode)
        {
            robotDrive = new SwerveDrive(this);
        }
        //
        // Create and initialize other subsystems.
        //
        if (RobotParams.Preferences.useSubsystems)
        {
        }
        //
        // Miscellaneous.
        //
        if (pdp != null)
        {
            pdp.registerEnergyUsedForAllUnregisteredChannels();
        }

        if (RobotParams.Preferences.hybridMode)
        {
            // Hybrid mode uses command-based, let's enable command-based support.
            TrcTaskMgr.TaskObject robotPeriodicTask = TrcTaskMgr.createTask(
                "RobotPeriodicTask", this::robotPeriodicTask);
            robotPeriodicTask.registerTask(TaskType.POST_PERIODIC_TASK);
        }
        //
        // Create Robot Modes.
        //
        setupRobotModes(new FrcTeleOp(this), new FrcAuto(this), new FrcTest(this), new FrcDisabled(this));
    }   //robotInit

    /**
     * This method is called periodically after mode specific periodic method is called. This is to simulate the
     * robotPeriodic method in TimedRobot.
     *
     * @param taskType specifies the type of task being run. This may be useful for handling multiple task types.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void robotPeriodicTask(TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        // Runs the Command Based Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands, and running
        // subsystem periodic() methods.  This must be called from the robot's periodic block in order for anything
        // in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }   //robotPeriodicTask

    /**
     * This method is called to prepare the robot before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    @Override
    public void robotStartMode(RunMode runMode, RunMode prevMode)
    {
        final String funcName = "robotStartMode";
        //
        // Read FMS Match info.
        //
        FrcMatchInfo matchInfo = FrcMatchInfo.getMatchInfo();
        //
        // Start trace logging.
        //
        if (runMode != RunMode.DISABLED_MODE && RobotParams.Preferences.useTraceLog)
        {
            openTraceLog(matchInfo);
            setTraceLogEnabled(true);
        }
        globalTracer.traceInfo(
            funcName, "[%.3f] %s: ***** %s *****", TrcTimer.getModeElapsedTime(), matchInfo.eventDate, runMode);
        //
        // Start subsystems.
        //
        if (robotDrive != null)
        {
            robotDrive.startMode(runMode, prevMode);
        }
        ledIndicator.reset();
    }   //robotStartMode

    /**
     * This method is called to prepare the robot right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    @Override
    public void robotStopMode(RunMode runMode, RunMode nextMode)
    {
        final String funcName = "robotStopMode";
        //
        // Stop subsystems.
        //
        if (robotDrive != null)
        {
            robotDrive.stopMode(runMode, nextMode);
        }
        ledIndicator.reset();
        //
        // Performance status report.
        //
        if (battery != null)
        {
            double totalEnergy = battery.getTotalEnergy();
            globalTracer.traceInfo(
                funcName, "TotalEnergy=%.3fWh (%.2f%%)",
                totalEnergy, totalEnergy * 100.0 / RobotParams.BATTERY_CAPACITY_WATT_HOUR);
        }

        if (runMode != RunMode.DISABLED_MODE)
        {
            printPerformanceMetrics(globalTracer);
        }
        //
        // Stop trace logging.
        //
        setTraceLogEnabled(false);
        closeTraceLog();
    }   //robotStopMode

    /**
     * This method is called periodically to update various hardware/subsystem status of the robot to the dashboard
     * and trace log. In order to lower the potential impact these updates, this method will only update the dashboard
     * at DASHBOARD_UPDATE_INTERVAL.
     */
    public void updateStatus()
    {
        final String funcName = "updateStatus";
        double currTime = TrcTimer.getModeElapsedTime();
        RunMode runMode = getCurrentRunMode();

        if (currTime >= nextDashboardUpdateTime)
        {
            int lineNum = 8;

            nextDashboardUpdateTime = currTime + RobotParams.DASHBOARD_UPDATE_INTERVAL;
            if (RobotParams.Preferences.showPowerConsumption)
            {
                if (pdp != null)
                {
                    dashboard.putNumber("Power/pdpTotalCurrent", pdp.getTotalCurrent());
                    dashboard.putNumber("Power/totalEnergy", battery.getTotalEnergy());
                    dashboard.putData("Power/pdpInfo", pdp.getPdpSendable());
                    if (runMode == RunMode.TELEOP_MODE)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] Battery: currVoltage=%.2f, lowestVoltage=%.2f",
                            currTime, battery.getVoltage(), battery.getLowestVoltage());
                        globalTracer.traceInfo(funcName, "[%.3f] Total=%.2fA", currTime, pdp.getTotalCurrent());
                    }
                }
            }

            if (RobotParams.Preferences.showDriveBase)
            {
                if (robotDrive != null)
                {
                    TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();

                    dashboard.putNumber("DriveBase/xPos", robotPose.x);
                    dashboard.putNumber("DriveBase/yPos", robotPose.y);
                    dashboard.putData("DriveBase/heading", ((FrcAHRSGyro) robotDrive.gyro).getGyroSendable());
                    dashboard.putNumber("DriveBase/Yaw", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getYaw());
                    dashboard.putNumber("DriveBase/Pitch", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getPitch());
                    dashboard.putNumber("DriveBase/Roll", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getRoll());
                    dashboard.putNumber("DriveBase/AccelX", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getWorldLinearAccelX());
                    dashboard.putNumber("DriveBase/AccelY", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getWorldLinearAccelY());
                    dashboard.putNumber("DriveBase/AccelZ", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getWorldLinearAccelZ());
                    dashboard.putNumber("DriverBase/Compass", ((FrcAHRSGyro) robotDrive.gyro).ahrs.getCompassHeading());
                    //
                    // DriveBase debug info.
                    //
                    double lfDriveEnc = robotDrive.driveMotors[RobotDrive.INDEX_LEFT_FRONT].getPosition();
                    double rfDriveEnc = robotDrive.driveMotors[RobotDrive.INDEX_RIGHT_FRONT].getPosition();
                    double lbDriveEnc = robotDrive.driveMotors[RobotDrive.INDEX_LEFT_BACK] != null?
                                            robotDrive.driveMotors[RobotDrive.INDEX_LEFT_BACK].getPosition(): 0.0;
                    double rbDriveEnc = robotDrive.driveMotors[RobotDrive.INDEX_RIGHT_BACK] != null?
                                            robotDrive.driveMotors[RobotDrive.INDEX_RIGHT_BACK].getPosition(): 0.0;

                    dashboard.displayPrintf(
                        lineNum++, "DriveEnc: lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f, avg=%.0f",
                        lfDriveEnc, rfDriveEnc, lbDriveEnc, rbDriveEnc,
                        (lfDriveEnc + rfDriveEnc + lbDriveEnc + rbDriveEnc) / 4.0);

                    if (robotDrive instanceof SwerveDrive)
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robotDrive;

                        dashboard.displayPrintf(
                            lineNum++, "SteerAngle: lf=%.1f/%.1f, rf=%.1f/%.1f, lb=%.1f/%.1f, rb=%.1f/%.1f",
                            swerveDrive.swerveModules[RobotDrive.INDEX_LEFT_FRONT].getSteerAngle(),
                            swerveDrive.swerveModules[RobotDrive.INDEX_LEFT_FRONT].steerMotor.getMotorPosition(),
                            swerveDrive.swerveModules[RobotDrive.INDEX_RIGHT_FRONT].getSteerAngle(),
                            swerveDrive.swerveModules[RobotDrive.INDEX_RIGHT_FRONT].steerMotor.getMotorPosition(),
                            swerveDrive.swerveModules[RobotDrive.INDEX_LEFT_BACK].getSteerAngle(),
                            swerveDrive.swerveModules[RobotDrive.INDEX_LEFT_BACK].steerMotor.getMotorPosition(),
                            swerveDrive.swerveModules[RobotDrive.INDEX_RIGHT_BACK].getSteerAngle(),
                            swerveDrive.swerveModules[RobotDrive.INDEX_RIGHT_BACK].steerMotor.getMotorPosition());
                    }
                    dashboard.displayPrintf(lineNum++, "DriveBase: pose=%s", robotPose);

                    if (RobotParams.Preferences.showPidDrive)
                    {
                        TrcPidController xPidCtrl = robotDrive.pidDrive.getXPidCtrl();
                        if (xPidCtrl != null)
                        {
                            xPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        robotDrive.pidDrive.getYPidCtrl().displayPidInfo(lineNum);
                        lineNum += 2;
                        robotDrive.pidDrive.getTurnPidCtrl().displayPidInfo(lineNum);
                        lineNum += 2;
                    }
                }
            }

            if (RobotParams.Preferences.showVision)
            {
                if (limeLightVision != null)
                {
                    FrcLimeLightVision.DetectedObject object = limeLightVision.getDetectedObject();
                    dashboard.displayPrintf(lineNum++, "LimeLight: obj=%s", object);
                }

                if (photonVision != null)
                {
                    FrcPhotonVision.DetectedObject object = photonVision.getBestDetectedObject();
                    dashboard.displayPrintf(lineNum++, "Photon: obj=%s", object);
                }

                if (openCvVision != null)
                {
                    TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> object =
                        openCvVision.getDetectedTargetInfo(null, null);
                    dashboard.displayPrintf(lineNum++, "OpenCv: %s", object);
                }
            }

            if (RobotParams.Preferences.showSubsystems)
            {
            }
        }
    }   //updateStatus

    /**
     * This method creates and opens the trace log with the file name derived from the given match info.
     * Note that the trace log is disabled after it is opened. The caller must explicitly call setTraceLogEnabled
     * to enable/disable it.
     *
     * @param matchInfo specifies the match info from which the trace log file name is derived.
     */
    public void openTraceLog(FrcMatchInfo matchInfo)
    {
        if (RobotParams.Preferences.useTraceLog && !traceLogOpened)
        {
            String fileName = matchInfo.eventName != null?
                String.format(Locale.US, "%s_%s%03d", matchInfo.eventName, matchInfo.matchType, matchInfo.matchNumber):
                getCurrentRunMode().name();

            traceLogOpened = TrcDbgTrace.openTraceLog(RobotParams.TEAM_FOLDER_PATH + "/tracelogs", fileName);
        }
    }   //openTraceLog

    /**
     * This method closes the trace log if it was opened.
     */
    public void closeTraceLog()
    {
        if (traceLogOpened)
        {
            TrcDbgTrace.closeTraceLog();
            traceLogOpened = false;
        }
    }   //closeTraceLog

    /**
     * This method enables/disables the trace log.
     *
     * @param enabled specifies true to enable trace log, false to disable.
     */
    public void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogOpened)
        {
            TrcDbgTrace.setTraceLogEnabled(enabled);
        }
    }   //setTraceLogEnabled

    //
    // Getters for sensor data.
    //

    /**
     * This method returns the pressure value from the pressure sensor.
     *
     * @return pressure value.
     */
    public double getPressure()
    {
        return pressureSensor != null? (pressureSensor.getVoltage() - 0.5) * 50.0: 0.0;
    }   //getPressure

}   //class Robot
