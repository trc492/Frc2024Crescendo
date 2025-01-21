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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import edu.wpi.first.wpilibj.DriverStation;
import frclib.driverio.FrcChoiceMenu;
import frclib.driverio.FrcMatchInfo;
import frclib.driverio.FrcUserChoices;
import team492.autocommands.CmdAuto;
import team492.commandbased.exampleAuto;
import trclib.command.CmdPidDrive;
import trclib.command.CmdPurePursuitDrive;
import trclib.command.CmdTimedDrive;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcRobot.RunMode;

/**
 * This class implements the code to run in Autonomous Mode.
 */
public class FrcAuto implements TrcRobot.RobotMode
{
    private static final String moduleName = "FrcAuto";
    //
    // Global constants.
    //

    //
    // Auto choices enums.
    //
    public enum AutoStrategy
    {
        CRESCENDO_AUTO,
        HYBRID_MODE_AUTO,
        PP_DRIVE,
        PID_DRIVE,
        TIMED_DRIVE,
        DO_NOTHING
    }   //enum AutoStrategy

    public enum AutoStartPos
    {
        SW_SOURCE_SIDE(0),
        SW_CENTER(1),
        SW_AMP_SIDE(2),
        AMP(3);
        // The value can be used as index into arrays if necessary.
        int value;
        AutoStartPos(int value)
        {
            this.value = value;
        }   //AutoStartPos
    }   //enum AutoStartPos

    public enum ScoreWingNotes
    {
        SCORE_NONE,
        SCORE_ONE,
        SCORE_TWO,
        SCORE_THREE
    }   //enum ScoreWingNotes

    public enum EndAction
    {
        JUST_STOP,
        PARK_NEAR_CENTER_LINE,
        HOARD_ONE_NOTE,
        SCORE_ONE_NOTE,
        SCORE_TWO_NOTES
    }   //enum EndAction

    /**
     * This class encapsulates all user choices for autonomous mode from the smart dashboard.
     *
     * To add an autonomous choice, follow the steps below:
     * 1. Add a DBKEY string constant.
     * 2. If the choice is a choice menu, create a FrcChoiceMenu variable for it, create the enum type if necessary,
     *    add code to create the FrcChoiceMenu object and add choices to it.
     * 3. Call userChoices to add the new choice object and provide default value if necessary.
     * 4. Add a getter method for the new choice.
     * 5. Add an entry of the new choice to the toString method.
     */
    public static class AutoChoices
    {
        // Smart dashboard keys for Autonomous choices.
        private static final String DBKEY_AUTO_ALLIANCE = "Auto/Alliance";
        private static final String DBKEY_AUTO_STRATEGY = "Auto/Strategy";
        private static final String DBKEY_AUTO_START_POS = "Auto/StartPos";
        private static final String DBKEY_AUTO_SCORE_WING_NOTES = "Auto/ScoreWingNotes";
        private static final String DBKEY_AUTO_END_ACTION = "Auto/EndAction";
        private static final String DBKEY_AUTO_RELOCALIZE = "Auto/Relocalize";

        private static final String DBKEY_AUTO_START_DELAY = "Auto/StartDelay";
        private static final String DBKEY_AUTO_PATHFILE = "Auto/PathFile";
        private static final String DBKEY_AUTO_X_DRIVE_DISTANCE = "Auto/XDriveDistance";
        private static final String DBKEY_AUTO_Y_DRIVE_DISTANCE = "Auto/YDriveDistance";
        private static final String DBKEY_AUTO_TURN_ANGLE = "Auto/TurnAngle";
        private static final String DBKEY_AUTO_DRIVE_TIME = "Auto/DriveTime";
        private static final String DBKEY_AUTO_DRIVE_POWER = "Auto/DrivePower";

        private final FrcUserChoices userChoices = new FrcUserChoices();
        private final FrcChoiceMenu<DriverStation.Alliance> allianceMenu;
        private final FrcChoiceMenu<AutoStrategy> strategyMenu;
        private final FrcChoiceMenu<AutoStartPos> startPosMenu;
        private final FrcChoiceMenu<ScoreWingNotes> scoreWingNotesMenu;
        private final FrcChoiceMenu<EndAction> endActionMenu;

        public AutoChoices()
        {
            //
            // Create autonomous mode specific choice menus.
            //
            allianceMenu = new FrcChoiceMenu<>(DBKEY_AUTO_ALLIANCE);
            strategyMenu = new FrcChoiceMenu<>(DBKEY_AUTO_STRATEGY);
            startPosMenu = new FrcChoiceMenu<>(DBKEY_AUTO_START_POS);
            scoreWingNotesMenu = new FrcChoiceMenu<>(DBKEY_AUTO_SCORE_WING_NOTES);
            endActionMenu = new FrcChoiceMenu<>(DBKEY_AUTO_END_ACTION);
            //
            // Populate autonomous mode choice menus.
            //
            allianceMenu.addChoice("Red", DriverStation.Alliance.Red);
            allianceMenu.addChoice("Blue", DriverStation.Alliance.Blue, true, true);

            if (RobotParams.Preferences.hybridMode)
            {
                strategyMenu.addChoice("Hybrid-mode Auto", AutoStrategy.HYBRID_MODE_AUTO);
            }
            else
            {
                strategyMenu.addChoice("Crescendo Auto", AutoStrategy.CRESCENDO_AUTO, true, false);
                strategyMenu.addChoice("Pure Pursuit Drive", AutoStrategy.PP_DRIVE);
                strategyMenu.addChoice("PID Drive", AutoStrategy.PID_DRIVE);
                strategyMenu.addChoice("Timed Drive", AutoStrategy.TIMED_DRIVE);
            }
            strategyMenu.addChoice("Do Nothing", AutoStrategy.DO_NOTHING, false, true);

            startPosMenu.addChoice("Start Position: Amp", AutoStartPos.AMP, true, false);
            startPosMenu.addChoice("Start Position: Subwoofer amp-side", AutoStartPos.SW_AMP_SIDE);
            startPosMenu.addChoice("Start Position: Subwoofer center", AutoStartPos.SW_CENTER);
            startPosMenu.addChoice("Start Position: Subwoofer source-side", AutoStartPos.SW_SOURCE_SIDE, false, true);

            scoreWingNotesMenu.addChoice("Score None", ScoreWingNotes.SCORE_NONE);
            scoreWingNotesMenu.addChoice("Score One", ScoreWingNotes.SCORE_ONE);
            scoreWingNotesMenu.addChoice("Score Two", ScoreWingNotes.SCORE_TWO);
            scoreWingNotesMenu.addChoice("Score Three", ScoreWingNotes.SCORE_THREE, true, true);

            endActionMenu.addChoice("Just Stop", EndAction.JUST_STOP);
            endActionMenu.addChoice("Park", EndAction.PARK_NEAR_CENTER_LINE);
            endActionMenu.addChoice("Hoard One Note", EndAction.HOARD_ONE_NOTE);
            endActionMenu.addChoice("Score One Note", EndAction.SCORE_ONE_NOTE);
            endActionMenu.addChoice("Score Two Notes", EndAction.SCORE_TWO_NOTES, true, true);
            //
            // Initialize dashboard with default choice values.
            //
            userChoices.addChoiceMenu(DBKEY_AUTO_ALLIANCE, allianceMenu);
            userChoices.addChoiceMenu(DBKEY_AUTO_STRATEGY, strategyMenu);
            userChoices.addChoiceMenu(DBKEY_AUTO_START_POS, startPosMenu);
            userChoices.addChoiceMenu(DBKEY_AUTO_SCORE_WING_NOTES, scoreWingNotesMenu);
            userChoices.addChoiceMenu(DBKEY_AUTO_END_ACTION, endActionMenu);
            userChoices.addBoolean(DBKEY_AUTO_RELOCALIZE, false);

            userChoices.addNumber(DBKEY_AUTO_START_DELAY, 0.0);
            userChoices.addString(DBKEY_AUTO_PATHFILE, "DrivePath.csv");
            userChoices.addNumber(DBKEY_AUTO_X_DRIVE_DISTANCE, 6.0);    // in feet
            userChoices.addNumber(DBKEY_AUTO_Y_DRIVE_DISTANCE, 6.0);    // in feet
            userChoices.addNumber(DBKEY_AUTO_TURN_ANGLE, 90.0);         // in degrees
            userChoices.addNumber(DBKEY_AUTO_DRIVE_TIME, 4.0);          // in seconds
            userChoices.addNumber(DBKEY_AUTO_DRIVE_POWER, 0.5);
        }   //AutoChoices

        //
        // Getters for autonomous mode choices.
        //

        public DriverStation.Alliance getAlliance()
        {
            // Get alliance info from FMS if one is connected. If not, get it from dashboard.
            FrcMatchInfo matchInfo = FrcMatchInfo.getMatchInfo();
            return matchInfo.eventName != null? matchInfo.alliance: allianceMenu.getCurrentChoiceObject();
        }   //getAlliance

        public AutoStrategy getStrategy()
        {
            return strategyMenu.getCurrentChoiceObject();
        }   //getStrategy

        public AutoStartPos getStartPos()
        {
            return startPosMenu.getCurrentChoiceObject();
        }   //getStartPos

        public ScoreWingNotes getScoreWingNotes()
        {
            return scoreWingNotesMenu.getCurrentChoiceObject();
        }   //getScoreWingNotes

        public EndAction getEndAction()
        {
            return endActionMenu.getCurrentChoiceObject();
        }   //getEndAction

        public boolean getRelocalize()
        {
            return userChoices.getUserBoolean(DBKEY_AUTO_RELOCALIZE);
        }   //getRelocalize

        public double getStartDelay()
        {
            return userChoices.getUserNumber(DBKEY_AUTO_START_DELAY);
        }   //getStartDelay

        public String getPathFile()
        {
            return userChoices.getUserString(DBKEY_AUTO_PATHFILE);
        }   //getPathFile

        public double getXDriveDistance()
        {
            return userChoices.getUserNumber(DBKEY_AUTO_X_DRIVE_DISTANCE);
        }   //getXDriveDistance

        public double getYDriveDistance()
        {
            return userChoices.getUserNumber(DBKEY_AUTO_Y_DRIVE_DISTANCE);
        }   //getYDriveDistance

        public double getTurnAngle()
        {
            return userChoices.getUserNumber(DBKEY_AUTO_TURN_ANGLE);
        }   //getTurnAngle

        public double getDriveTime()
        {
            return userChoices.getUserNumber(DBKEY_AUTO_DRIVE_TIME);
        }   //getDriveTime

        public double getDrivePower()
        {
            return userChoices.getUserNumber(DBKEY_AUTO_DRIVE_TIME);
        }   //getDrivePower

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "alliance=\"%s\" " +
                "strategy=\"%s\" " +
                "startPos=\"%s\" " +
                "scoreWingNotes=\"%s\" " +
                "endAction=\"%s\" " +
                "relocalize=\"%s\" " +
                "startDelay=%.0f sec " +
                "pathFile=\"%s\" " +
                "xDistance=%.1f ft " +
                "yDistance=%.1f ft " +
                "turnDegrees=%.0f deg " +
                "driveTime=%.0f sec " +
                "drivePower=%.1f",
                getAlliance(), getStrategy(), getStartPos(), getScoreWingNotes(), getEndAction(), getRelocalize(),
                getStartDelay(), getPathFile(), getXDriveDistance(), getYDriveDistance(), getTurnAngle(),
                getDriveTime(), getDrivePower());
        }   //toString

    }   //class AutoChoices

    //
    // Global objects.
    //

    public static final AutoChoices autoChoices = new AutoChoices();
    private final Robot robot;
    private TrcRobot.RobotCommand autoCommand;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object to access all robot hardware and subsystems.
     */
    public FrcAuto(Robot robot)
    {
        //
        // Create and initialize global objects.
        //
        this.robot = robot;
    }   //FrcAuto

    /**
     * This method checks if an autonomous command is running.
     *
     * @return true if autonomous command is running, false otherwise.
     */
    public boolean isAutoActive()
    {
        return autoCommand != null && autoCommand.isActive();
    }   //isAutoActive

    /**
     * This method cancels the autonomous command if one is running.
     */
    public void cancel()
    {
        if (autoCommand != null)
        {
            autoCommand.cancel();
            autoCommand = null;
        }
    }   //cancel

    //
    // Implements TrcRobot.RunMode.
    //

    /**
     * This method is called when the autonomous mode is about to start. Typically, you put code that will prepare
     * the robot for start of autonomous here such as creating autonomous command according to the chosen autonomous
     * strategy, initializing autonomous command and enabling/configuring sensors and subsystems that are necessary
     * for the autonomous command.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Retrieve Auto choices.
        //
        robot.globalTracer.logInfo(moduleName, "MatchInfo", "%s", FrcMatchInfo.getMatchInfo());
        robot.globalTracer.logInfo(moduleName, "AutoChoices", "%s", autoChoices);
        //
        // Create autonomous command.
        //
        switch (autoChoices.getStrategy())
        {
            case CRESCENDO_AUTO:
                if (robot.robotDrive != null)
                {
                    autoCommand = new CmdAuto(robot, autoChoices);
                }
                break;

            case HYBRID_MODE_AUTO:
                robot.m_autonomousCommand = new exampleAuto(robot.robotDrive);
                // schedule the autonomous command (example)
                if (robot.m_autonomousCommand != null)
                {
                    robot.m_autonomousCommand.schedule();
                }
                break;

            case PP_DRIVE:
                if (robot.robotDrive != null)
                {
                    autoCommand = new CmdPurePursuitDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.xPosPidCoeff, robot.robotDrive.yPosPidCoeff,
                        robot.robotDrive.turnPidCoeff, robot.robotDrive.velPidCoeff);
                    ((CmdPurePursuitDrive) autoCommand).start(
                        0.0, false, RobotParams.TEAM_FOLDER_PATH + "/" + autoChoices.getPathFile(), false);
                }
                break;

            case PID_DRIVE:
                if (robot.robotDrive != null)
                {
                    autoCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive);
                    ((CmdPidDrive) autoCommand).start(
                        autoChoices.getStartDelay(), autoChoices.getDrivePower(), null,
                        new TrcPose2D(
                            autoChoices.getXDriveDistance()*12.0,
                            autoChoices.getYDriveDistance()*12.0,
                            autoChoices.getTurnAngle()));
                }
                break;

            case TIMED_DRIVE:
                if (robot.robotDrive != null)
                {
                    autoCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, autoChoices.getStartDelay(), autoChoices.getDriveTime(), 0.0,
                        autoChoices.getDrivePower(), 0.0);
                }
                break;

            default:
            case DO_NOTHING:
                autoCommand = null;
                break;
        }
    }   //startMode

    /**
     * This method is called when autonomous mode is about to end. Typically, you put code that will do clean
     * up here such as canceling unfinished autonomous command and disabling autonomous sensors and subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Stop autonomous command.
        //
        if (autoCommand != null)
        {
            autoCommand.cancel();
        }
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (autoCommand != null)
        {
            //
            // Run the autonomous command.
            //
            autoCommand.cmdPeriodic(elapsedTime);
        }

        if (slowPeriodicLoop)
        {
            //
            // Update robot status.
            //
            if (RobotParams.Preferences.doStatusUpdate)
            {
                robot.updateStatus();
            }
        }
    }   //periodic

}   //class FrcAuto
