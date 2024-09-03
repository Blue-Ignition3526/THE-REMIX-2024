package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import lib.BlueShift.constants.CTRECANDevice;
import lib.BlueShift.constants.PIDFConstants;
import lib.BlueShift.constants.SwerveModuleOptions;
import lib.BlueShift.utils.SwerveChassis;
import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.util.PIDConstants;

public final class Constants {
    public static final class Logging {
        public static final boolean kDebug = true;
    }

    public static final class SwerveDrive {
        // * Gyro
        public static final CTRECANDevice kGyroDevice = new CTRECANDevice(34, "*");

        // * Controller
        public static final double kJoystickDeadband = 0.1;

        // * Heading Controller
        public static final Measure<Angle> kHeadingTolerance = Degrees.of(3);

        // * Physical model of the robot
        public static final class PhysicalModel {
            // * MAX DISPLACEMENT SPEED (and acceleration)
            public static Measure<Velocity<Distance>> kMaxSpeed = MetersPerSecond.of(4);
            public static final Measure<Velocity<Velocity<Distance>>> kMaxAcceleration = MetersPerSecondPerSecond.of(kMaxSpeed.in(MetersPerSecond));

            // * MAX ROTATIONAL SPEED (and acceleration)
            public static final Measure<Velocity<Angle>> kMaxAngularSpeed = RotationsPerSecond.of(1);
            public static final Measure<Velocity<Velocity<Angle>>> kMaxAngularAcceleration = RotationsPerSecond.per(Second).of(Math.pow(kMaxAngularSpeed.in(RotationsPerSecond), 2));

            // * Drive wheel diameter
            public static final Measure<Distance> kWheelDiameter = Inches.of(4);

            // * Gear ratios
            public static final double kDriveMotorGearRatio = 1.0 / 6.12; // 6.12:1 Drive
            public static final double kTurningMotorGearRatio = 1.0 / 12.8; // 12.8:1 Steering

            // * Conversion factors (Drive Motor)
            public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * (kWheelDiameter.in(Meters) / 2) * 2 * Math.PI;
            public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0;

            // * Conversion factors (Turning Motor)
            public static final double kTurningEncoder_RotationToRadian = kTurningMotorGearRatio * 2.0 * Math.PI;
            public static final double kTurningEncoder_RPMToRadianPerSecond = kTurningEncoder_RotationToRadian / 60.0;

            // * Robot Without bumpers measures
            public static final Measure<Distance> kTrackWidth = Inches.of(23.08);
            public static final Measure<Distance> kWheelBase = Inches.of(22.64);

            // * Robot with bumpers
            public static final Measure<Distance> kWidthWithBumpers = Meters.of(0.56);
            public static final Measure<Distance> kLengthWithBumpers = Meters.of(0.56);

            // * Extension margins
            public static final class ExtensionMargins {
                public static final Measure<Distance> kFrontExtension = Meters.of(0.3);
                public static final Measure<Distance> kBackExtension = Meters.of(0);
                public static final Measure<Distance> kLeftExtension = Meters.of(0);
                public static final Measure<Distance> kRightExtension = Meters.of(0);
            }
    
            // * Create a kinematics instance with the positions of the swerve modules
            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(SwerveChassis.sizeToModulePositions(kTrackWidth.in(Meters), kWheelBase.in(Meters)));

            // * Physical constants
            public static final double kRobotMassKg = 46;
        }

        // * Swerve modules configuration
        public static final class SwerveModules {
            // * PID
            public static final PIDFConstants kTurningPIDConstants = new PIDFConstants(0.5);

            // * Swerve modules options
            public static final SwerveModuleOptions kFrontLeftOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(11, "*"))
                .setDriveMotorID(22)
                .setTurningMotorID(21)
                .setName("Front Left");

            public static final SwerveModuleOptions kFrontRightOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(12, "*"))
                .setDriveMotorID(24)
                .setTurningMotorID(23)
                .setName("Front Right");

            public static final SwerveModuleOptions kBackLeftOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(13, "*"))
                .setDriveMotorID(26)
                .setTurningMotorID(25)
                .setName("Back Left");

            public static final SwerveModuleOptions kBackRightOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(14, "*"))
                .setDriveMotorID(28)
                .setTurningMotorID(27)
                .setName("Back Right");
        }

        // * AUTONOMOUS
        public static final class Autonomous {
            public static final PIDConstants kTranslatePIDConstants = new PIDConstants(1.45, 0.5, 0.0);
            public static final PIDConstants kRotatePIDConstants = new PIDConstants(10, 0.0, 0.0);
            public static final Measure<Velocity<Distance>> kMaxSpeedMetersPerSecond = MetersPerSecond.of(1);
        }
    }

    // * FIELD
    public static final class Field {
        public static final Pose2d kInitialPoseMeters = new Pose2d(new Translation2d(1, 2), new Rotation2d(0, 0));
        public static final Pose2d kBlueSpeakerPoseMeters = new Pose2d(new Translation2d(0, 5.55), Rotation2d.fromDegrees(0));
        public static final Pose2d kRedSpeakerPoseMeters = new Pose2d(new Translation2d(17, 5.55), Rotation2d.fromDegrees(0));
    }

    // * VISION
    public static final class Vision {
        public static final class Limelight3G {
            public static final String kName = "limelight-threeg";
            public static final int kOdometryPipeline = 0;
            public static final int kSpeakerPipeline = 1;
            public static final int kViewfinderPipeline = 2;
        }

        public static final class Limelight3 {
            public static final String kName = "limelight-three";
            public static final int kNotePipeline = 0;
            public static final int kOdometryPipeline = 1;
            public static final int kViewfinderPipeline = 2;
        }

        public static final class LimelightTwoPlus {
            public static final String kName = "limelight-twoplus";
            public static final int kOdometryPipeline = 0;
            public static final int kViewfinderPipeline = 1;
        }

        public static final class Arducam_Left {
            public static final String kName = "Arducam_Left";
            public static final Transform3d kRobotToCamera = new Transform3d(-0.61d, 0d, 0.6d, new Rotation3d(0, 0, 5d));
        }

        public static final class Arducam_Right {
            public static final String kName = "Arducam_Right";
            public static final Transform3d kRobotToCamera = new Transform3d(0.61d, 0d, 0.6d, new Rotation3d(0, 0, -5d));
        }
    }

    //* INTAKE
    public static final class Intake {
        public static final class Rollers {
            // * Intake motor config
            public static final int kintakeRollersMotorID = 36;
    
            // * Intake gear ratio
            public static final double kIntakeRollersGearRatio = 1d / 5d;
    
            // * Speeds
            public static final double kIntakeOutSpeed = -0.5;
            public static final double kIntakeInSpeed = 0.5;
            public static final double kIntakeHoldSpeed = 0.05;
            public static final double kGiveToShooterSpeed = -0.6;

            // * Intake times
            public static final double kMaxOuttakeTime = 3;
            public static final double kMaxIntakeTime = 6;

            // * Lifter limits
            public static final int kPieceSwitchPort = 1;
        }

        public static final class Lifter {
            // * Lifter motor config
            public static final int kLifterMotorID = 37;

            // * Speeds
            public static final double kMaxLifterVoltage = 7;
            public static final double kMinLifterSpeed =  -kMaxLifterVoltage;
    
            // * Lifter encoder 
            public static final int kLifterEncoderPort = 0;
            public static final double kLifterEncoderOffset = 0.367;

            public static final class PhysicalModel {
                // * Motion
                public static final Constraints kLifterConstraints = new Constraints(36, 20);
                public static final ProfiledPIDController kLifterPIDController = new ProfiledPIDController(2.5, 0.0, 0.0, kLifterConstraints);

                // * Angles
                public static final Measure<Angle> kShooterAngle = Degrees.of(-110);
                public static final Measure<Angle> kAmplifierPrevAngle = Degrees.of(40);
                public static final Measure<Angle> kAmplifierFinalAngle = Degrees.of(85);
                public static final Measure<Angle> kGroundAngle = Degrees.of(65);
            }
        }
    }

    // * SHOOTER
    public static final class Shooter {
        // Shooter motor config
        public static final int kLeftShooterMotorID = 30;
        public static final int kRightShooterMotorID = 31;

        // Shooter motor rpm conversion
        public static final double kShooterGearRatio = 1.0 / 1.0;

        // Shooter speeds
        public static final double kShooterMaxSpeed = 1;
        public static final double kShooterSpeakerLeftSpeed = kShooterMaxSpeed;
        public static final double kShooterSpeakerRightSpeed = kShooterMaxSpeed;

        // Shooter motor time
        public static final double kMaxShootTime = 4;
    }

    // * CLIMBER
    public static final class Climber {
        // Climber motor config
        public static final int kLeftClimberMotorID = 33;
        public static final int kRightClimberMotorID = 32;

        // Climber speed
        public static final double kClimberUpSpeed = 0.9;
        public static final double kClimberDownSpeed = -0.75;

        // Max current (Used for reseting the climber)
        public static final double kMaxCurrent = 20;
    }
}
