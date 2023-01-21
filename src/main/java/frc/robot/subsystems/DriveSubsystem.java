package frc.robot.subsystems;

import frc.robot.RobotMap;

public class DriveSubsystem {

    private static final double IN_TO_M = .0254;
    private static final int MOTOR_ENCODER_CODES_PER_REV = 2048;
    private static final double DIAMETER_INCHES = 5.0;
    private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M;
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    private static final double GEAR_RATIO = 12.75;
    private static final double TICKS_PER_METER = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
    private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;
 
    private static final WPI_TalonFX leftFrontMotor = RobotMap.leftFrontDriveMotor;
    private static final WPI_TalonFX leftBackMotor = RobotMap.leftBackDriveMotor;
    private static final WPI_TalonFX rightFrontMotor = RobotMap.rightFrontDriveMotor;
    private static final WPI_TalonFX rightBackMotor = RobotMap.rightBackDriveMotor;


    public DriveSubsystem() {
        resetEncoders();

        leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
        rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID());

        leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftFrontMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        leftFrontMotor.configvelocityMeasurementWindow(16);
        leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

        leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftBackMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        leftBackMotor.configvelocityMeasurementWindow(16);
        leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

        rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightFrontMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        rightFrontMotor.configvelocityMeasurementWindow(16);
        rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

        rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
        rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightBackMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        rightBackMotor.configvelocityMeasurementWindow(16);
        rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

        leftFrontMotor.setNeutralMode(NeutralMode.Coast);
        leftBackMotor.setNeutralMode(NeutralMode.Coast);
        rightBackMotor.setNeutralMode(NeutralMode.Coast);
        rightFrontMotor.setNeutralMode(NeutralMode.Coast);
        
        leftFrontMotor.setInverted(false);
        leftBackMotor.setInverted(false);
        rightFrontMotor.setInverted(false);
        rightBackMotor.setInverted(false);  

        leftFrontMotor.configNominalOutputForward(0, 10);
        leftFrontMotor.configNominalOutputReverse(0, 10);
        leftFrontMotor.configPeakOutputForward(1, 10);
        leftFrontMotor.configPeakOutputReverse(-1, 10);

        rightFrontMotor.configNominalOutputForward(0, 10);
        rightFrontMotor.configNominalOutputReverse(0, 10);
        rightFrontMotor.configPeakOutputForward(1, 10);
        rightFrontMotor.configPeakOutputReverse(-1, 10);

        leftBackMotor.configNominalOutputForward(0, 10);
        leftBackMotor.configNominalOutputReverse(0, 10);
        leftBackMotor.configPeakOutputForward(1, 10);
        leftBackMotor.configPeakOutputReverse(-1, 10);

        rightBackMotor.configNominalOutputForward(0, 10);
        rightBackMotor.configNominalOutputReverse(0, 10);
        rightBackMotor.configPeakOutputForward(1, 10);
        rightBackMotor.configPeakOutputReverse(-1, 10);

        leftBackMotor.setSensorPhase(true);
        leftFrontMotor.setSensorPhase(true);
        rightBackMotor.setSensorPhase(false);
        rightFrontMotor.setSensorPhase(false);

        rightBackMotor.setInverted(false);
        rightFrontMotor.setInverted(false);
        leftFrontMotor.setInverted(true);
        leftBackMotor.setInverted(true);
    }

    public void resetEncoders() {
        rightBackMotor.setSelectedSensorPosition(0);
        rightFrontMotor.setSelectedSensorPosition(0);
        leftBackMotor.setSelectedSensorPosition(0);
        leftFrontMotor.setSelectedSensorPosition(0);
    }

    public static void drive(double throttle, double rotate) {
        leftFrontMotor.set(throttle + rotate);
        rightFrontMotor.set(throttle - rotate);
        leftBackMotor.set(throttle + rotate);
        rightBackMotor.set(throttle - rotate);
    }

    public double getRightBackEncoderPosition() {
        return rightBackMotor.getSelectedSensorPosition();
    }

    public double getLeftBackEncoderPosition() {
        return leftBackMotor.getSelectedSensorPosition();
    }

    public double distanceTravelledinTicks() {
        return (getLeftBackEncoderPosition() + getRightBackEncoderPosition()) / 2;
    }

    public double getLeftBackEncoderVelocityMetersPerSecond() {
        double leftVelocityMPS = (leftBackMotor.getSelectedSensorVelocity()*10);
        leftVelocityMPS = leftVelocityMPS * METERS_PER_TICKS;
        return (leftVelocityMPS);
    }

    public double leftDistanceTravelledInMeters() {
        double left_dist = getLeftBackEncoderPosition() * METERS_PER_TICKS;
        return left_dist;
    }

    public void stop() {
        drive(0, 0);
    }
    
    public void setModePercentVoltage() {
        leftFrontMotor.set(ControlMode.PercentOutput, 0);
        rightFrontMotor.set(ControlMode.PercentOutput, 0);
        leftBackMotor.set(ControlMode.PercentOutput, 0);
        rightBackMotor.set(ControlMode.PercentOutput, 0);
    }

}