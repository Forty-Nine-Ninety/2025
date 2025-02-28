
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.config.HolonomicPathFollowerConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.ReplanningConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    //public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

    public static class Ports
    {
        public static int PORT_JOYSTICK_DRIVE = 0;
        public static int PORT_JOYSTICK_OPERATOR = 1;

        //CAN IDs

        //Swerve occupies 1-8

        public static final int CAN_ELEVATOR_BOTTOM_LEFT = 9;
        public static final int CAN_ELEVATOR_BOTTOM_RIGHT = 10;
        public static final int CAN_ELEVATOR_TOP_LEFT = 11;
        public static final int CAN_ELEVATOR_TOP_RIGHT = 12;
        
        public static final int CAN_L1SHOOTER = 13;

        public static final int CAN_L2L3SHOOTER_LEFT = 14;
        public static final int CAN_L2L3SHOOTER_RIGHT = 15;

        public static final int PORT_DIO_BREAK_BEAM = 0; // for l2l3 subsystem -chloe

        //public static final int CAN_TRAPDOOR = 16;
        //public static final int CAN_STINGER_ROTATION = 17;
        //public static final int CAN_ELEVATOR_CLIPS = 18;
    
    public static class MotorConfig
    {
        public static double CANANDCODER_RESOLUTION = 16384.0;
    }

    public static class RobotMeasurements
    {
        public static double DRIVETRAIN_WHEEL_DIAMETER_IN = 4.0;
        public static final double DRIVE_GEAR_RATIO = 5.9; //previously 6.75
        public static final double STEERING_GEAR_RATIO = 150.0/7.0;

        public static double ARM_MOTION_REDUCTION = 1000.0/3.0;
        
        public static double ROBOT_RADIUS = 0.47;
        public static double WHEEL_SPEED = 4.60; //Meters per sec
        //public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
        //public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    }

    public static class OurUnits
    {
        //Base units
        public static Unit METER = new BaseUnit(Dimension.Length, 1d);
        public static Unit KILOMETER = new BaseUnit(Dimension.Length, METER.getScalar() * 1000d);
        public static Unit FEET = new BaseUnit(Dimension.Length, METER.getScalar() * 3.280839895d);
        public static Unit INCH = new BaseUnit(Dimension.Length, FEET.getScalar() / 12d);

        public static Unit SECOND = new BaseUnit(Dimension.Time, 1d);
        public static Unit MINUTE = new BaseUnit(Dimension.Time, SECOND.getScalar() * 60d);
        public static Unit HOUR = new BaseUnit(Dimension.Time, MINUTE.getScalar() * 60d);
        public static Unit MILLISECOND = new BaseUnit(Dimension.Time, SECOND.getScalar() / 1000d);
        //public static Unit ENCODER_TIME = new BaseUnit(Dimension.Time, MILLISECOND.getScalar() * 100d);
                
        public static Unit KILOGRAM = new BaseUnit(Dimension.Mass, 1d);

        public static Unit RADIAN = new BaseUnit(Dimension.Angle, 1d);
        public static Unit REVOLUTION = new BaseUnit(Dimension.Angle, RADIAN.getScalar() * 2d * Math.PI);
        public static Unit DEGREE = new BaseUnit(Dimension.Angle, REVOLUTION.getScalar() / 360d);
        // Angle represented by encoder ticks, i.e 16384 ticks is a full revolution
        public static Unit ENCODER_ANGLE = new BaseUnit(Dimension.Angle, REVOLUTION.getScalar() / MotorConfig.CANANDCODER_RESOLUTION);

        public static Unit AMPERE = new BaseUnit(Dimension.Current, 1d);

        //Compound units
        //public static Unit ENCODER_VELOCITY_UNIT = new CompoundUnit(ENCODER_ANGLE, ENCODER_TIME);
        public static Unit ENCODER_ANGULAR_VELOCITY = new CompoundUnit(ENCODER_ANGLE, SECOND);

        public static Unit ANGULAR_VELOCITY = new CompoundUnit(RADIAN, SECOND);
        public static Unit METERS_PER_SECOND = new CompoundUnit(METER, SECOND);

        public static Unit METERS_PER_SECOND_2 = new CompoundUnit(METERS_PER_SECOND, SECOND);
        public static Unit NEWTON = new CompoundUnit(new Unit[] {KILOGRAM, METERS_PER_SECOND_2}, new Unit[] {});
        public static Unit JOULE = new CompoundUnit(new Unit[] {NEWTON, METER}, new Unit[] {});
        public static Unit COULOMB = new CompoundUnit(new Unit[] {AMPERE, SECOND}, new Unit[] {});
        public static Unit VOLTAGE = new CompoundUnit(JOULE, COULOMB);
    }

    public static class SubsystemConfig
    {
        //Movement information

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class MotionControl
    {
        //PID
        public static final PIDConstants ELEVATOR_tbd_PID = new PIDConstants(1, 1, 1);
        public static final PIDConstants L1SHOOTER_tbd_PID = new PIDConstants(1, 1, 1);
        public static final double CLOSED_LOOP_RAMP_RATE = 1.0;
        public static final double OPEN_LOOP_RAMP_RATE = 1.0;
        public static final double ELEVATOR_FEEDFORWARD = 1; //tbd
    }

    public static final class AutonConstants
    {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
        public static final HolonomicPathFollowerConfig autoBuilderPathConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        //new PIDConstants(5.0, 0.0 ,0.2), //original p = 5, 1st attempt: p = 5, d = 0.5, 2nd attempt: p= 5, d = 0.5, 3rd attempt: p = 5, d = 3 this caused the wheels to shutter
        //new PIDConstants(1.5, 0.0, 0), //5.0, 0, 0.2

        RobotMeasurements.WHEEL_SPEED, // Max module speed, in m/s
        RobotMeasurements.ROBOT_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig());

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  
        public static final TrapezoidProfile.Constraints kDriveControllerConstraints = new TrapezoidProfile.Constraints(
        RobotMeasurements.ROBOT_RADIUS, RobotMeasurements.WHEEL_SPEED);
  

        public static final double maxAutoSpeed = 1.4;
        public static final double maxAutoAcceleration = 2.0;

        public static final HashMap<String, Command> eventMap = new HashMap<>();
        
    }

    public static final class VisionConstants
    {
        //Vision, in m, m/s, rad/s
        public static final double tagToNodeDistance = 0.17;
        public static final double xMarginOfError = 0.05;
        public static final double yMarginOfError = 0.03;
        public static final double rotationMarginOfError = 0.1;
        public static final double translationSpeed = 3;
        public static final double rotationSpeed = 1;
    }

    public static class DriveSettings
    {
        // Joystick Deadband
        public static final double LEFT_X_DEADBAND  = 0.1;
        public static final double LEFT_Y_DEADBAND  = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT    = 6;
        public static final double ARM_DEADBAND    = 0.1;


        //Driver settings
        public static double JOYSTICK_THROTTLE_X_EXPONENT = 2.0;
        public static double JOYSTICK_THROTTLE_Y_EXPONENT = 2.0;
        public static double JOYSTICK_TURNING_EXPONENT = 1.0;
        public static double ARCADE_SPEED_X_MULTIPLIER = 1.0;
        public static double ARCADE_SPEED_Y_MULTIPLIER = -1.0;
        public static double ARCADE_ROTATION_MULTIPLIER = 1.0;
        public static double ARCADE_ROTATIONV2_MULTIPLIER = 0.6;
    }
}
}