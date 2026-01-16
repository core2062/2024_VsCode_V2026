package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import frc.lib.util.COREConstants;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static double kObjectTrackingVal = 0.02;

    public static boolean kTarget = false;
    public static boolean kOverride = false;
    public static double kDesiredAngle = 0.0;

    public static int AutoSelected = 0;
    public static String side = "";

    public static boolean assemblyDone = false;
    public static boolean endAssembly1 = false;
    public static boolean AimDone = false;
    public static final class Swerve {
        public static final int pigeonID = 0;

        public static COREConstants SpeedMod = new COREConstants("Drive Speed Modifier", 0.8);
        
        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(24.5); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 5.0; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(234.31);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-192.04);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(200.30);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-280.45);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static RobotConfig config = null;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    public static final class LauncherConstants {
        public static final int kUpperMotorPort = 8;
        public static final int kLowerMotorPort = 9;
        
        public static final int kLeftSideMotorPort = 10;//TODO: undetermined amount, extra two might not be used
        public static final int kRightSideMotorPort = 11;

        public static final int kLeftRotationMotorPort = 12;
        public static final int kRightRotationMotorPort = 13;

        public static final int kLauncherEncoder = 4;

        public static final COREConstants kSpeakerCloseAngle = new COREConstants("Speaker Close Angle", 50);

        public static final COREConstants kSpeakerLaunchSpeed = new COREConstants("Speaker Launcher Speed", 0.6); //TODO: needs to be tuned
        public static final COREConstants kAMPLaunchSpeed = new COREConstants("AMP Launcher Speed", 0.2); //TODO: needs to be tuned
        public static final COREConstants kFeedSpeed = new COREConstants("feed Speed", 0.5); //TODO: needs to be tuned
        public static final COREConstants kDelay = new COREConstants("Laucnher Delay", 0.5); //TODO: needs to be tuned

        public static final COREConstants kLeftRotationSpeed = new COREConstants("Left Rotation Speed", 0.8); //TODO: needs to be tuned
        public static final COREConstants kRightRotationSpeed = new COREConstants("Right Rotation Speed", 0.8); //TODO: needs to be tuned

        /** Launcher Rotation Motor Constants */
        public static double MotorPos = 0;

        public static final int motorContinuousCurrentLimit = 25;
        public static final int motorPeakCurrentLimit = 40;
        public static final int motorPeakCurrentDuration = 100;
        public static final boolean motorEnableCurrentLimit = true;
  
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;

        public static final double motorOpenloopRamp = 0.25;
        public static final double motorClosedloopRamp = 0.55;

        public static LauncherRotationMotorConfigs motorConfigs = new LauncherRotationMotorConfigs();
    }

    public static final class IntakeConstants {

        public static final int kIntakeMotorPort = 14;//TODO: planned on one, may be more

        public static final COREConstants kIntakeSpeed = new COREConstants("intake Speed", 0.8); //TODO: needs to be tuned
    }

    public static class VisionConstants{
        public static int SpeakerID = 4;
        public static int farSpeakerID = 0;
        public static int AmpID = 0;

        public static double DesiredAngle = 0.0;
    }

    public static class ClimberConstants{
        public static final int kClimberMotorPort = 15;
        public static final COREConstants kUpClimberSpeed = new COREConstants("Climber down Speed", -0.75);
        public static final COREConstants kDownClimberSpeed = new COREConstants("Climber up Speed",0.75);
    }
}
