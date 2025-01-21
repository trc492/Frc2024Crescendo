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

package team492.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import frclib.motor.FrcCANSparkMax;
import frclib.motor.FrcCANTalonFX;
import team492.RobotParams;
import trclib.dataprocessor.TrcUtil;
import trclib.subsystem.TrcShooter;

public class Shooter
{
    private static final String moduleName = Shooter.class.getSimpleName();

    private static Shooter instance = null;
    private final FrcCANTalonFX shooterMotor;
    private final FrcCANSparkMax tiltMotor;
    private final WPI_PigeonIMU pigeonIMU;
    private final TrcShooter shooter;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param shootOp specifies the interface to call for shooting the object.
     */
    public Shooter(TrcShooter.ShootOperation shootOp)
    {
        shooterMotor = new FrcCANTalonFX(moduleName + ".shooterMotor", RobotParams.Shooter.shooterCandId);
        shooterMotor.resetFactoryDefault();
        shooterMotor.setMotorInverted(RobotParams.Shooter.shooterMotorInverted);
        shooterMotor.disableLowerLimitSwitch();
        shooterMotor.disableUpperLimitSwitch();
        shooterMotor.setBrakeModeEnabled(false);
        shooterMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        shooterMotor.enableMotionProfile(
            RobotParams.Shooter.shooterMaxVelocity, RobotParams.Shooter.shooterMaxAcceleration, 0.0);
        shooterMotor.setPositionSensorScaleAndOffset(RobotParams.Shooter.shooterPosScale, 0.0);
        shooterMotor.setVelocityPidParameters(
            RobotParams.Shooter.shooterVelPidCoeff, RobotParams.Shooter.shooterVelTolerance);
        shooterMotor.setPresets(
            true, RobotParams.Shooter.shooterPresetVelTolerance, RobotParams.Shooter.shooterPresetVelocities);

        tiltMotor = new FrcCANSparkMax(moduleName + ".tiltMotor", RobotParams.Shooter.tiltCanId, false, true);
        tiltMotor.resetFactoryDefault();
        tiltMotor.setMotorInverted(RobotParams.Shooter.tiltMotorInverted);
        tiltMotor.setBrakeModeEnabled(true);
        tiltMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        tiltMotor.enableLowerLimitSwitch(true);
        tiltMotor.enableUpperLimitSwitch(true);
        tiltMotor.setPositionSensorScaleAndOffset(
            RobotParams.Shooter.tiltPosScale, RobotParams.Shooter.tiltPosOffset, RobotParams.Shooter.tiltZeroOffset);
        // tiltMotor.resetPositionOnLowerLimitSwitch();
        // We are using software position PID control for Tilt. So we just enable software PID before setting
        // PID coefficients.
        tiltMotor.setSoftwarePidEnabled(true);
        tiltMotor.setPositionPidParameters(
            RobotParams.Shooter.tiltPosPidCoeff, RobotParams.Shooter.tiltPosPidTolerance);
        // Tilt is heavily geared down, so don't really need gravity compensation.
        // tiltMotor.setPositionPidPowerComp(this::getTiltGravityComp);
        tiltMotor.setPresets(
            false, RobotParams.Shooter.tiltPresetPosTolerance, RobotParams.Shooter.tiltPresetPositions);
        if (RobotParams.Preferences.usePigeonIMU)
        {
            pigeonIMU = new WPI_PigeonIMU(RobotParams.HWConfig.CANID_PIGEON_IMU);
            pigeonIMU.configFactoryDefault();
        }
        else
        {
            pigeonIMU = null;
        }

        TrcShooter.PanTiltParams tiltParams = new TrcShooter.PanTiltParams(
            RobotParams.Shooter.tiltPowerLimit, RobotParams.Shooter.tiltMinAngle, RobotParams.Shooter.tiltMaxAngle);
        shooter = new TrcShooter(moduleName, shooterMotor, null, tiltMotor, tiltParams, null, null);
        instance = this;
    }   //Shooter

    /**
     * This method returns the instance name.
     */
    @Override
    public String toString()
    {
        return shooter.toString();
    }   //toString

    /**
     * This method returns the Shooter parent object instance.
     *
     * @return shooter parent instance.
     */
    public static Shooter getInstance()
    {
        return instance;
    }   //getInstance

    /**
     * This method returns the Pigeon's yaw value.
     *
     * @return yaw value.
     */
    public static double getTilterYaw()
    {
        double value = 0.0;

        if (instance != null && instance.pigeonIMU != null)
        {
            value = instance.pigeonIMU.getYaw();
        }

        return value;
    }   //getTilterYaw

    /**
     * This method returns the Pigeon's pitch value.
     *
     * @return pitch value.
     */
    public static double getTilterPitch()
    {
        double value = 0.0;

        if (instance != null && instance.pigeonIMU != null)
        {
            value = instance.pigeonIMU.getPitch();
        }

        return value;
    }   //getTilterPitch

    /**
     * This method returns the Pigeon's roll value.
     *
     * @return roll value.
     */
    public static double getTilterRoll()
    {
        double value = 0.0;

        if (instance != null && instance.pigeonIMU != null)
        {
            value = instance.pigeonIMU.getRoll();
        }

        return value;
    }   //getTilterRoll

    /**
     * This method clears all tilter motor faults.
     */
    public static void clearTilterFaults()
    {
        if (instance != null)
        {
            instance.tiltMotor.motor.clearFaults();
        }
    }   //clearTilterFaults

    /**
     * This method returns the created TrcShooter object.
     *
     * @return TrcShooter object.
     */
    public TrcShooter getShooter()
    {
        return shooter;
    }   //getShooter

    // /**
    //  * This method is called by PID control to determine the power required to compensate for gravity in essence
    //  * making tilt gravity neutral (i.e. hold its position, aka feedforward).
    //  *
    //  * @param currPower specifies the current tilt power (not used).
    //  * @return gravity compensation power.
    //  */
    // private double getTiltGravityComp(double currPower)
    // {
    //     double gravityComp =
    //         RobotParams.Shooter.tiltMaxHoldingPower * Math.cos(Math.toRadians(shooter.getTiltAngle()));
    //     shooter.tracer.traceDebug(moduleName, "gravityComp=" + gravityComp);
    //     return gravityComp;
    // }   //getTiltGravityComp

}   //class Shooter
