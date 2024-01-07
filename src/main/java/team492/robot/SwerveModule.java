package team492.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import team492.Robot;
import team492.robot.lib.math.Conversions;
import team492.robot.lib.util.CTREModuleState;
import team492.robot.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    //private CANCoder angleEncoder;
    private Canandcoder angleEncoder;

    // private static final double driveMotorCPR = RobotParams.FALCON_CPR;
    // private static final double angleMotorCPR = RobotParams.FALCON_CPR;
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new Canandcoder(moduleConstants.angleEncoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
            // double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            // mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToMotorRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
            // double velocity = Conversions.MPSToMotorVel(
            //     desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio, driveMotorCPR);
            // mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, driveFeedForward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        anglePosition.Position = Conversions.degreesToMotorRot(angle.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.setControl(anglePosition);
        // mAngleMotor.set(ControlMode.Position, Conversions.degreesToMotorCounts(angle.getDegrees(), Constants.Swerve.angleGearRatio, angleMotorCPR));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.motorRotToDegrees(mAngleMotor.getPosition().getValue(), Constants.Swerve.angleGearRatio));
        // return Rotation2d.fromDegrees(Conversions.motorCountsToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio, angleMotorCPR));
    }

    public Rotation2d getAngleEncoderPos(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsPosition()*360);
    }

    public void resetToAbsolute(){
        double absolutePosition =
            Conversions.degreesToMotorRot(getAngleEncoderPos().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.getConfigurator().setPosition(absolutePosition);//setRotorPosition(absolutePosition);
        // double absolutePosition = Conversions.degreesToTalon(waitForCANcoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        // mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){
        // Do not clear zero calibration.
        angleEncoder.resetFactoryDefaults(false);
        //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();
        // mAngleMotor.configFactoryDefault();
        // mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        // mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        // mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        // // Make sure we give it time for the config to finish before sync'ing the encoder position.
        // Timer.delay(0.2);
        // resetToAbsolute();
    }

    private void configDriveMotor(){
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0);//setRotorPosition(0);
        // mDriveMotor.configFactoryDefault();
        // mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        // mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        // mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        // mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.motorRPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            // Conversions.motorVelToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio, driveMotorCPR),
            getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.motorRotToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            // Conversions.motorCountsToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio, driveMotorCPR),
            getAngle()
        );
    }
}
