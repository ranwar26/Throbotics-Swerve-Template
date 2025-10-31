// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class MotorConfigs {

    public class DriveConfigs {

        public static final SparkMaxConfig driveMotor = new SparkMaxConfig();
        public static final SparkMaxConfig turnMotor = new SparkMaxConfig();

        static {
            // Configure drive motor
            driveMotor
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(DriveConstants.driveMotorCurrentLimit)
                    .voltageCompensation(12.0);
            driveMotor.encoder
                    .positionConversionFactor(DriveConstants.driveEncoderPositionFactor)
                    .velocityConversionFactor(DriveConstants.driveEncoderVelocityFactor)
                    .uvwMeasurementPeriod(10)
                    .uvwAverageDepth(2);
            driveMotor.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pidf(
                            DriveConstants.driveKp, 0.0,
                            DriveConstants.driveKd, 0.0);
            driveMotor.signals
                    .primaryEncoderPositionAlwaysOn(true)
                    .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
                    .primaryEncoderVelocityAlwaysOn(true)
                    .primaryEncoderVelocityPeriodMs(20)
                    .appliedOutputPeriodMs(20)
                    .busVoltagePeriodMs(20)
                    .outputCurrentPeriodMs(20);

            // Configure turn motor
            turnMotor
                    .inverted(DriveConstants.turnInverted)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(DriveConstants.turnMotorCurrentLimit)
                    .voltageCompensation(12.0);
            turnMotor.absoluteEncoder
                    .inverted(DriveConstants.turnEncoderInverted)
                    .positionConversionFactor(DriveConstants.turnEncoderPositionFactor)
                    .velocityConversionFactor(DriveConstants.turnEncoderVelocityFactor)
                    .averageDepth(2);
            turnMotor.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(DriveConstants.turnPIDMinInput, DriveConstants.turnPIDMaxInput)
                    .pidf(DriveConstants.turnKp, 0.0, DriveConstants.turnKd, 0.0);
            turnMotor.signals
                    .absoluteEncoderPositionAlwaysOn(true)
                    .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
                    .absoluteEncoderVelocityAlwaysOn(true)
                    .absoluteEncoderVelocityPeriodMs(20)
                    .appliedOutputPeriodMs(20)
                    .busVoltagePeriodMs(20)
                    .outputCurrentPeriodMs(20);
        }
    }
}
