package team492.subsystems;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcCANFalcon;
import team492.Robot;
import team492.RobotParams;

public class Shooter {
    private static final String moduleName = Shooter.class.getSimpleName();

    private final FrcCANFalcon shooterMotor;
    private final FrcCANFalcon tilterMotor;

    public Shooter() {
        shooterMotor = new FrcCANFalcon(moduleName + ".motor", RobotParams.Shooter.shooterCandId);
        shooterMotor.resetFactoryDefault();
        shooterMotor.setMotorInverted(RobotParams.Shooter.shooterMotorInverted);
        shooterMotor.setBrakeModeEnabled(false);
        shooterMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);

        tilterMotor = new FrcCANFalcon(moduleName + ".motor", RobotParams.Shooter.tilterCanId);
        tilterMotor.resetFactoryDefault();
        tilterMotor.setMotorInverted(RobotParams.Shooter.tilterMotorInverted);
        tilterMotor.setBrakeModeEnabled(true);
        tilterMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);

        tilterMotor.enableLowerLimitSwitch(true);
        tilterMotor.enableUpperLimitSwitch(true);

        shooterMotor.setPositionSensorScaleAndOffset(RobotParams.Shooter.shooterScale, 0.0);
        tilterMotor.setPositionSensorScaleAndOffset(RobotParams.Shooter.tilterScale, RobotParams.Shooter.tilterOffset);

    }

    public void setShooterVelocity(String owner, double velocity) {
        shooterMotor.setVelocity(owner, 0.0, velocity, 0.0, null);
    }

    public void setShooterVelocity(double velocity) {
        shooterMotor.setVelocity(null, 0.0, velocity, 0.0, null);
    }

    public void setTilterPosition(String owner, double position, TrcEvent completionEvent, double timeout) {
        tilterMotor.setPosition(owner, 0, position, true, RobotParams.Shooter.shooterPowerLimit, completionEvent, timeout);
    }

    public void setTilterPosition(String owner, double position) {
        setTilterPosition(owner, position, null, 0.0);
    }

    public void setTilterPosition(double position) {
        setTilterPosition(null, position);
    }


    public double getShooterVelocity() {
        return shooterMotor.getVelocity();
    }

    public double getTilterPosition() {
        return tilterMotor.getPosition();
    }

    public void stopShooter() {
        shooterMotor.stop();
    }
}