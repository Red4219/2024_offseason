// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Tools.AutonomousDetail;
import frc.robot.Tools.Parts.PIDGains;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static boolean debugDriveTrain = true;
	public static boolean enableLogger = false;
	public static boolean debugPhotonVision = false;
	public static boolean kEnablePhotonVision = false;
	public static boolean kEnableLimelight = false;
	public static boolean kDebugLimelight = false;

	public static class ModuleConstants {

		// Current limits for the wheels
		public static final int kTurnMotorCurrentLimit = 25;
		public static final int kDriveMotorCurrentLimit = 35;

		// Constants set for the _SDS MK4i_ and MK4
		public static final double kdriveGearRatioL1 = 1d / 8.14;
		public static final double kdriveGearRatioL2 = 1d / 6.75;
		public static final double kdriveGearRatioL3 = 1d / 6.12;
		public static final double kdriveGearRatioL4 = 1d / 5.14;
		public static final double kturnGearRatio = 1d / (150d / 7d);

		public static final double kwheelCircumference = Units.inchesToMeters(4) * Math.PI;

		// The max speed the modules are capable of
		public static final double kMaxModuleSpeedMetersPerSecond = Units.feetToMeters(14.5);

		public static final double ksVolts = .1;
		public static final double kDriveFeedForward = .2;

		// TODO: Retune feedforward values for turning
		public static final double kvTurning = .43205;
		public static final double ksTurning = .17161; 

		// NEO drive motor CAN ID's
		public static final int kFrontLeftDriveMotorPort = 4;
		public static final int kFrontRightDriveMotorPort = 6;
		public static final int kRearLeftDriveMotorPort = 2;
		public static final int kRearRightDriveMotorPort = 8;

		// NEO turning motor CAN ID's
		public static final int kFrontLeftTurningMotorPort = 3;
		public static final int kFrontRightTurningMotorPort = 5;
		public static final int kRearLeftTurningMotorPort = 1;
		public static final int kRearRightTurningMotorPort = 7;

		// CANcoder CAN ID's
		public static final int kFrontLeftTurningEncoderPort = 10;
		public static final int kFrontRightTurningEncoderPort = 11;
		public static final int kRearLeftTurningEncoderPort = 9;
		public static final int kRearRightTurningEncoderPort = 12;

		// Offset angle for absolute encoders (find this using CTRE client)
		public static final double kFrontLeftAngleZero = -13.79;
		public static final double kFrontRightAngleZero = 90.5;
		public static final double kRearLeftAngleZero = -171.12;
		public static final double kRearRightAngleZero = 110.3;

		public static final PIDGains kModuleDriveGains = new PIDGains(.1, 0, 0);
		public static final PIDGains kModuleTurningGains = new PIDGains(6.5, 0.25, 0.15);
	}

	public static class DriveConstants {
		//public static final double kMaxSpeedMetersPerSecond = 4.5;

		// this sets turning speed (keep this low)
		//public static final double kMaxRPM = 10;
		public static final double kBumperToBumperWidth = Units.inchesToMeters(31);

		public static final double kTrackWidth = Units.inchesToMeters(32); // in meters!
		public static final double kWheelBase = Units.inchesToMeters(28); // in meters!

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // RL
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // RR

		public static final PIDGains kGyroTurningGains = new PIDGains(.025, 0, 0);
		public static final double kMaxTurningVelocityDegrees = 20;
		public static final double kMaxTurningAcceleratonDegrees = 10;
		public static final double kGyroTurnTolerance = 2;

		public static enum kDriveModes {
			NORMAL,
			AIM,
			LOCK_WHEELS
		}

		public static final double kChassisAutoAimRotation = 1.9;
	}

	public static class PoseDefinitions {
		public static enum kFieldPoses {
			AMPLIFIER,
			SOURCE
		}

		public static final Pose2d kAmplifierPoseRed = new Pose2d(14.73, 7.69, Rotation2d.fromDegrees(90.0));
		public static final Pose2d kAmplifierPoseBlue = new Pose2d(1.83, 7.78, Rotation2d.fromDegrees(90.0));
		public static final Pose2d kSourcePoseRed = new Pose2d(0.98, 1.05, Rotation2d.fromDegrees(-120.16));
		public static final Pose2d kSourcePoseBlue = new Pose2d(15.35, 0.88, Rotation2d.fromDegrees(-120.0));
	}

	/**
	 * The constants pertaining to Autonoumus
	 */
	public static class AutoConstants {

		public static class PathPLannerConstants {

			// PID constants for path planner (these control drive direction not reaching
			// target wheel speeds)
			public static final PIDConstants kPPDriveConstants = new PIDConstants(8.5, 0, 0);
			public static final PIDConstants kPPTurnConstants = new PIDConstants(3.5, 0, 0);

			public static final double kPPMaxVelocity = 4.00;
			public static final double kPPMaxAcceleration = 2.50;

			public static final double kMaxModuleSpeed = 4.5; // Max module speed, in m/s
			public static final double kDriveBaseRadius = 0.4; // Drive base radius in meters. Distance from robot center to furthest module.
		}

		public static final PIDGains kTurnCommandGains = new PIDGains(.004, 0.0003, 0);
		public static final double kTurnCommandToleranceDeg = 0.5;
		public static final double kTurnCommandRateToleranceDegPerS = 0;

		public static final double kAimTargetTolerance = 2.0;

	}

	/**
	 * The constants pertaining to the drive station
	 */
	public static class OperatorConstants {
		public static final int kDriveJoystickPort = 0;
		public static final int kTurnJoystickPort = 1;
		public static final int kOperatorControllerPort = 2;
		public static final int kProgrammerControllerPort = 3;

		public static final double KDeadBand = .125;
		// this is the number that the joystick input will be raised to
		public static final double kJoystickPow = 2.5;
	}

	public static class PhotonVisionConstants {

		public static final boolean VisionEnabled = true;
		//public static final boolean PhysicalCamera = false;

		public static double TagHeight = Units.inchesToMeters(6.5);
		public static double TagWidth = Units.inchesToMeters(6.5);

		//public static final PoseStrategy poseStrategy = PoseStrategy.AVERAGE_BEST_TARGETS;
		public static final PoseStrategy poseStrategy = PoseStrategy.CLOSEST_TO_REFERENCE_POSE;

		public static double camDiagFOV = 170.0;
		public static double camPitch = 0.0;
		public static double camHeightOffGround = Units.inchesToMeters(6.0);
		// the side to side position of the camera relative to the robot center
		public static double camX = Units.inchesToMeters(-15.0);
		// the front to back position of the camera relative to the robot center
		public static double camY = Units.inchesToMeters(0.0);

		public static Transform3d cameraToRobot = new Transform3d(
                    new Translation3d(
						camX,
						camY,
					 	PhotonVisionConstants.camHeightOffGround
					),
					new Rotation3d(
						0,
						PhotonVisionConstants.camPitch,
						0
					)
				);

		//public static final String CameraName = "Microsoft_LifeCam_HD-3000";
		//public static final String CameraName = "OV5647";
		public static final String CameraName = "cam1";

		// Simulated Vision System.
    	// Configure these to match your PhotonVision Camera,
    	// pipeline, and LED setup.
		public static double sim_camDiagFOV = camDiagFOV; // degrees - assume wide-angle camera
		public static double sim_camPitch = camPitch; // degrees
    	public static double sim_camHeightOffGround = camHeightOffGround; // meters
    	public static double sim_maxLEDRange = 20; // meters
    	public static int sim_camResolutionWidth = 640; // pixels
    	public static int sim_camResolutionHeight = 480; // pixels
    	//public static double sim_minTargetArea = 10; // square pixels
		public static double sim_minTargetArea = 300; // square pixels

		/**
    	* Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
    	* less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
    	*/
		public static double visionMeasurementStdDevsX = 0.5;
		public static double visionMeasurementStdDevsY = 0.5;
		public static double visionMeasurementStdDevsTheta = Units.degreesToRadians(10);
	}
	
	public static final String kRioCANBusName = "rio";
	public static final String kCanivoreCANBusName = "canivore";
	public static final String logFolders = "/media/sda2/";

	// Autonomous section
	public static Map<String, AutonomousDetail> AutonomousRoutines = Map.of(
		"Test Path 1", 
		new AutonomousDetail(
			"Test Path 1", 
			2.0,
			1.09,
			2.0,
			8.1026 - 1.09,
			.2,
			.2
		)
	);
}
