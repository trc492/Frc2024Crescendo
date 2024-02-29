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

import TrcCommonLib.trclib.TrcShooter;
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcCANSparkMax;
import team492.RobotParams;

public class Shooter
{
    private static final String moduleName = Shooter.class.getSimpleName();

    private final FrcCANFalcon shooterMotor;
    private final FrcCANSparkMax tiltMotor;
    private final TrcShooter shooter;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param shootOp specifies the interface to call for shooting the object.
     */
    public Shooter(TrcShooter.ShootOperation shootOp)
    {
        shooterMotor = new FrcCANFalcon(moduleName + ".shooterMotor", RobotParams.Shooter.shooterCandId);
        shooterMotor.resetFactoryDefault();
        shooterMotor.setMotorInverted(RobotParams.Shooter.shooterMotorInverted);
        shooterMotor.disableLowerLimitSwitch();
        shooterMotor.disableUpperLimitSwitch();
        shooterMotor.setBrakeModeEnabled(false);
        shooterMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        shooterMotor.enableMotionProfile(
            RobotParams.Shooter.shooterMaxVelocity, RobotParams.Shooter.shooterMaxAcceleration, 0.0);
        shooterMotor.setPositionSensorScaleAndOffset(RobotParams.Shooter.shooterPosScale, 0.0);
        shooterMotor.setVelocityPidCoefficients(RobotParams.Shooter.shooterVelPidCoeff);
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
        // We are using software position PID control for Tilt. So we just enable software PID before setting
        // PID coefficients.
        tiltMotor.setSoftwarePidEnabled(true);
        tiltMotor.setPositionPidCoefficients(RobotParams.Shooter.tiltPosPidCoeff);
        tiltMotor.setPositionPidTolerance(RobotParams.Shooter.tiltPosPidTolerance);
        // Tilt is heavily geared down, so don't really need gravity compensation.
        // tiltMotor.setPositionPidPowerComp(this::getTiltGravityComp);
        tiltMotor.setPresets(
            false, RobotParams.Shooter.tiltPresetPosTolerance, RobotParams.Shooter.tiltPresetPositions);

        TrcShooter.PanTiltParams tiltParams = new TrcShooter.PanTiltParams(
            RobotParams.Shooter.tiltPowerLimit, RobotParams.Shooter.tiltMinAngle, RobotParams.Shooter.tiltMaxAngle);
        shooter = new TrcShooter(moduleName, shooterMotor, tiltMotor, tiltParams, null, null);
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
