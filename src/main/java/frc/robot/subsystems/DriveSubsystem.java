package frc.robot.subsystems;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Mechanisms.SwerveModule;
import frc.robot.Tools.AutonomousDetail;
import frc.robot.Tools.GyroIONavX;
import frc.robot.Tools.Limelight;
import frc.robot.Tools.PhotonVision;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants.kDriveModes;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.Mechanisms.LED;
import frc.robot.Mechanisms.LED.LEDStatus;


public class DriveSubsystem extends SubsystemBase {

	private boolean fieldRelative = true;
	private boolean gyroTurning = false;
	private double targetRotationDegrees;

	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule rearLeft;
	private final SwerveModule rearRight;

	private SwerveModulePosition[] swervePosition;
	private SwerveModuleState[] swerveModuleStates;

	// Initalizing the gyro sensor
	private final AHRS gyro = new AHRS(SPI.Port.kMXP);

	private double xSpeed = 0.0;
	private double ySpeed = 0.0;
	private double rot = 0.0;

	private kDriveModes mode = kDriveModes.NORMAL;
	//private int speakerTarget = 0;
	private boolean targetLocked = false;
	private boolean isSim = false;
	private LimelightHelpers.PoseEstimate limelightMeasurement = null;

	// Odeometry class for tracking robot pose
	private SwerveDriveOdometry odometry;

	private Pose2d estimatedPose = new Pose2d();
	private double[] combinedEstimatedPoseArray = new double[9];
	private boolean setupAuto = false;
	private Pose2d startPosition = null;
	private boolean limeLightCanSeeTag = false;
	private boolean photonVisionCanSeeTag = false;


	// test for auto positioning
	private HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
		new PIDController(1, 0, 0),
		new PIDController(1, 0, 0),
		new ProfiledPIDController(
			2,
			0,
			0,
			new TrapezoidProfile.Constraints(
			12.0,
			6.0
			)
		)
	);

	private Trajectory trajectory = null;
	private Trajectory.State goal = null;

	// PID controller for gyro turning
	private ProfiledPIDController gyroTurnPidController = null;

	private SwerveDrivePoseEstimator poseEstimator = null;

	private PhotonVision _photonVision = null;
	private Limelight _limeLight = null;
	private Pose2d photonPose2d = null;

	private double autoX_Position = 0.0;
	private double autoY_Position = 0.0;
	private boolean autoPositionStatusX = false;
	private boolean autoPositionStatusY = false;

	private SwerveModuleState[] swerveModuleStatesRobotRelative;
	private EstimatedRobotPose phoneEstimatedRobotPose;

	/**
	* Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
    * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
    */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
    /**
    * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
    * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
    */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(
		Constants.PhotonVisionConstants.visionMeasurementStdDevsX, 
		Constants.PhotonVisionConstants.visionMeasurementStdDevsY, 
		Constants.PhotonVisionConstants.visionMeasurementStdDevsTheta
	);

	public AHRS getGyro() {
		return this.gyro;
	}

	private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {

		if(Constants.kEnablePhotonVision) {
			_photonVision = RobotContainer.photonVision;
		}

		if(Constants.kEnableLimelight) {
			_limeLight = RobotContainer.limelight;
		}

		if(RobotBase.isReal()) {
			isSim = false;
		} else {
			isSim = true;
		}
		
		gyro.reset();
		//_gyroIONavX = new GyroIONavX(gyro);

		frontLeft = new SwerveModule(
				"FL",
				ModuleConstants.kFrontLeftDriveMotorPort,
				ModuleConstants.kFrontLeftTurningMotorPort,
				ModuleConstants.kFrontLeftTurningEncoderPort,
				ModuleConstants.kFrontLeftAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains,
				true,
				false

			);

		frontRight = new SwerveModule(
				"FR",
				ModuleConstants.kFrontRightDriveMotorPort,
				ModuleConstants.kFrontRightTurningMotorPort,
				ModuleConstants.kFrontRightTurningEncoderPort,
				ModuleConstants.kFrontRightAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains,
				true,
				false
			);

		rearLeft = new SwerveModule(
				"RL",
				ModuleConstants.kRearLeftDriveMotorPort,
				ModuleConstants.kRearLeftTurningMotorPort,
				ModuleConstants.kRearLeftTurningEncoderPort,
				ModuleConstants.kRearLeftAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains,
				true,
				false
			);

		rearRight = new SwerveModule(
				"RR",
				ModuleConstants.kRearRightDriveMotorPort,
				ModuleConstants.kRearRightTurningMotorPort,
				ModuleConstants.kRearRightTurningEncoderPort,
				ModuleConstants.kRearRightAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains,
				true,
				false
			);

		swervePosition = new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()
		};

		odometry = new SwerveDriveOdometry(
				DriveConstants.kDriveKinematics,
				gyro.getRotation2d().unaryMinus(),
				swervePosition);

		gyroTurnPidController = new ProfiledPIDController(
				DriveConstants.kGyroTurningGains.kP,
				DriveConstants.kGyroTurningGains.kI,
				DriveConstants.kGyroTurningGains.kD,
				new TrapezoidProfile.Constraints(
						DriveConstants.kMaxTurningVelocityDegrees,
						DriveConstants.kMaxTurningAcceleratonDegrees));

		gyroTurnPidController.enableContinuousInput(-180, 180);
		gyroTurnPidController.setTolerance(DriveConstants.kGyroTurnTolerance);

		poseEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.kDriveKinematics,
				gyro.getRotation2d().unaryMinus(),
				swervePosition,
				new Pose2d(),
				stateStdDevs,
				visionMeasurementStdDevs
			);
		
		targetRotationDegrees = 0;

		if(Constants.debugDriveTrain == true) {

			// auto tab stuff
			ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
			autoTab.addDouble("AutoX Position", this::getAutoX_Position);
			autoTab.addDouble("AutoY Position", this::getAutoY_Position);
			autoTab.addBoolean("AutoX Status", this::getAutoPositionStatusX);
			autoTab.addBoolean("AutoY Status", this::getAutoPositionStatusY);
			autoTab.addString("Alliance", this::getAlliance);

			// gyro tab stuff
			ShuffleboardTab gyroTab = Shuffleboard.getTab("Gyro");
			gyroTab.addDouble("Yaw", gyro::getYaw);
			gyroTab.addDouble("Pitch", gyro::getPitch);
			gyroTab.addDouble("Roll", gyro::getRoll);

			// Swerve tab stuff
			/*ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
			swerveTab.addDouble("FL Absolute", frontLeft::getAbsoluteHeading);
			swerveTab.addDouble("FR Absolute", frontRight::getAbsoluteHeading);
			swerveTab.addDouble("RL Absolute", rearLeft::getAbsoluteHeading);
			swerveTab.addDouble("RR Absolute", rearRight::getAbsoluteHeading);
			swerveTab.addDouble("FL Meters", frontLeft::getDistanceMeters);
			swerveTab.addDouble("FR Meters", frontRight::getDistanceMeters);
			swerveTab.addDouble("RL Meters", rearLeft::getDistanceMeters);
			swerveTab.addDouble("RR Meters", rearRight::getDistanceMeters);
			swerveTab.addBoolean("Auto Aim", this::autoAim);
			swerveTab.addBoolean("Target Locked", this::getTargetLocked);*/
		}

		gyro.reset();

		poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
	}

	public void setupAuto(boolean setupAuto) {
		this.setupAuto = setupAuto;
	}

	public void setStartPosition(Pose2d startPosition) {
		this.startPosition = startPosition;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		updateOdometry();

		if(Constants.debugDriveTrain == true) {
			//SmartDashboard.putNumber("FL Offset Check", frontLeft.getAbsoluteHeading() + frontLeft.angleZero);
			//SmartDashboard.putNumber("FR Offset Check", frontRight.getAbsoluteHeading() + frontRight.angleZero);
			//SmartDashboard.putNumber("RL Offset Check", rearLeft.getAbsoluteHeading() + rearLeft.angleZero);
			//SmartDashboard.putNumber("RR Offset Check", rearRight.getAbsoluteHeading() + rearRight.angleZero);
			SmartDashboard.putNumber("2D X", getPose().getX());
			SmartDashboard.putNumber("2D Y", getPose().getY());
			SmartDashboard.putNumber("2D Gyro", -odometry.getPoseMeters().getRotation().getDegrees());
			SmartDashboard.putData("field", RobotContainer.field);
		}

		//Logger.recordOutput("Odometry/Robot", odometry.getPoseMeters());

		// Show the estimated position
		estimatedPose = poseEstimator.getEstimatedPosition();
		Logger.recordOutput("Estimator/Robot", estimatedPose);

		combinedEstimatedPoseArray[0] = estimatedPose.getX();
		combinedEstimatedPoseArray[1] = estimatedPose.getY();
		combinedEstimatedPoseArray[2] = estimatedPose.getRotation().getDegrees();
		Logger.recordOutput("Estimator/PoseArray", combinedEstimatedPoseArray);
	}

	// region getters
	public double getHeading() {
		if(isSim) {
			return gyro.getRotation2d().getDegrees();
		}
		return gyro.getRotation2d().unaryMinus().getDegrees();
	}

	public double getHeading360() {
		if(isSim) {
			return (gyro.getRotation2d().getDegrees() % 360);
		}
		return (gyro.getRotation2d().unaryMinus().getDegrees() % 360);
	}

	public double getRoll() {
		return gyro.getRoll();
	}

	public double getPitch() {
		return gyro.getPitch();
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public Pose2d getPoseEstimatorPose2d() {
		return poseEstimator.getEstimatedPosition();
	}

	public void resetOdometry(Pose2d pose) {

		odometry.resetPosition(
			gyro.getRotation2d(),
			swervePosition,
			pose
		);

		poseEstimator.resetPosition(
			gyro.getRotation2d(),
			swervePosition,
			pose
		);

		if(Constants.kEnablePhotonVision) {
			_photonVision.setReferencePose(pose);
		}

		//Logger.recordOutput("Odometry/Robot", odometry.getPoseMeters());
		//Logger.recordOutput("Estimator/Robot", poseEstimator.getEstimatedPosition());
	}
	
	public void lockWheels() {

		swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				new ChassisSpeeds(0, 0, 0));

		SwerveDriveKinematics.desaturateWheelSpeeds(
				swerveModuleStates, 0);

		setModuleStates(swerveModuleStates);
	}

	/*public void robotCentricDrive(double xSpeed, double ySpeed, double rot) {
		setFieldCentric(false);
		drive(xSpeed, ySpeed, rot);
		setFieldCentric(true);
	}*/

	/*public void drive(double xSpeed, double ySpeed, double rot) {
		drive(xSpeed, ySpeed, rot, false, false);
	}*/

	public void drive(double xSpeed, double ySpeed, double rot) {

		// Apply deadbands to inputs
		xSpeed *= ModuleConstants.kMaxModuleSpeedMetersPerSecond;
		ySpeed *= ModuleConstants.kMaxModuleSpeedMetersPerSecond;

		if (gyroTurning) {
			targetRotationDegrees += rot;
			rot = gyroTurnPidController.calculate(getHeading360(), targetRotationDegrees);
		} else {
			rot *= DriveConstants.kMaxRPM;
		}

		this.xSpeed = xSpeed;
		this.ySpeed = ySpeed;
		this.rot = rot;

		// If we are set to auto aim
		/*if (mode == kDriveModes.AIM) {
			if (Constants.kEnablePhotonVision) {
				if (_photonVision.canSeeTarget(speakerTarget) == true) {

					double targetYaw = _photonVision.aimAtTarget(speakerTarget);

					if (Math.abs(targetYaw) > Constants.AutoConstants.kAimTargetTolerance) {
						targetLocked = false;
						if (targetYaw > 0) {
							rot -= Constants.DriveConstants.kChassisAutoAimRotation;
						} else {
							rot += Constants.DriveConstants.kChassisAutoAimRotation;
						}
					} else {
						targetLocked = true;
					}
				}
			}
		} else if (mode == kDriveModes.LOCK_WHEELS) {
			lockWheels();
			return;
		}*/

		// We multiply (times) the rotation because it is inverted
		if(isSim) {
			swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()));
		} else {
			swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d().unaryMinus()));
		}
		
		setModuleStates(swerveModuleStates);
	}

	// This is for auto
	public ChassisSpeeds getChassisSpeedsRobotRelative() {
		if(isSim) {
			return ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());
		} 
		
		return ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d().unaryMinus());
	}

	// This is for auto
	public void setChassisSpeedsRobotRelative(ChassisSpeeds chassisSpeeds ) {

		chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

		swerveModuleStatesRobotRelative = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

		// In simulation, the actual navx does not work, so set the value from the chassisSpeeds
		if(isSim) {
			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
			SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			angle.set(angle.get() + -chassisSpeeds.omegaRadiansPerSecond);
		}

		frontLeft.setDesiredState(swerveModuleStatesRobotRelative[0]);
		frontRight.setDesiredState(swerveModuleStatesRobotRelative[1]);
		rearLeft.setDesiredState(swerveModuleStatesRobotRelative[2]);
		rearRight.setDesiredState(swerveModuleStatesRobotRelative[3]);

		Logger.recordOutput("SwerveModuleStates/Setpoints", swerveModuleStatesRobotRelative);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kMaxModuleSpeedMetersPerSecond);
		
		if(isSim) {

			ChassisSpeeds chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(
				desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3]
			);

			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
			SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			angle.set(angle.get() + -chassisSpeeds.omegaRadiansPerSecond);
		}

		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		rearLeft.setDesiredState(desiredStates[2]);
		rearRight.setDesiredState(desiredStates[3]);

		Logger.recordOutput("SwerveModuleStates/Setpoints", desiredStates);
	}

	public void updateOdometry() {
		swervePosition[0] = frontLeft.getPosition();
		swervePosition[1] = frontRight.getPosition();
		swervePosition[2] = rearLeft.getPosition();
		swervePosition[3] = rearRight.getPosition();

		if (Constants.kEnablePhotonVision) {

			phoneEstimatedRobotPose = _photonVision.getPose(poseEstimator.getEstimatedPosition());

			if(phoneEstimatedRobotPose != null) {

				if(phoneEstimatedRobotPose.targetsUsed.size() >= 1) {
					poseEstimator.addVisionMeasurement(
						phoneEstimatedRobotPose.estimatedPose.toPose2d(),
						//Timer.getFPGATimestamp() - phoneEstimatedRobotPose.timestampSeconds,
						phoneEstimatedRobotPose.timestampSeconds,
						visionMeasurementStdDevs
					);
					photonVisionCanSeeTag = true;

					// update the combined
					combinedEstimatedPoseArray[6] = phoneEstimatedRobotPose.estimatedPose.getX();
					combinedEstimatedPoseArray[7] = phoneEstimatedRobotPose.estimatedPose.getY();
					combinedEstimatedPoseArray[8] = phoneEstimatedRobotPose.estimatedPose.getRotation().toRotation2d().getDegrees();
				} else {
					photonVisionCanSeeTag = false;
				}
			} else {
				photonVisionCanSeeTag = false;
			}
		}

		if (Constants.kEnableLimelight) {

			/*_limeLight.setPose(odometry.getPoseMeters());

			limelightMeasurement = _limeLight.getPoseEstimate();*/

			//limelightMeasurement = _limeLight.getPose2d(odometry.getPoseMeters());
			limelightMeasurement = _limeLight.getPose2d(poseEstimator.getEstimatedPosition());

			// Did we get a measurement?
			if(limelightMeasurement != null && limelightMeasurement.tagCount >= 1) {

				/*if(!setupAuto) {
					RobotContainer.led1.setStatus(LEDStatus.ready);
				}*/

				limeLightCanSeeTag = true;

     			poseEstimator.addVisionMeasurement(
         			limelightMeasurement.pose,
         			limelightMeasurement.timestampSeconds
				);

				Logger.recordOutput("Limelight/Pose", limelightMeasurement.pose);
				//Logger.recordOutput("Limelight/position", _limeLight.getPoseArray());

				combinedEstimatedPoseArray[3] = limelightMeasurement.pose.getX();
				combinedEstimatedPoseArray[4] = limelightMeasurement.pose.getY();
				combinedEstimatedPoseArray[5] = limelightMeasurement.pose.getRotation().getDegrees();

				if(!gyro.isMoving() && limelightMeasurement.tagCount >= 1) {
					resetOdometry(limelightMeasurement.pose);
				}
			} else {
				//RobotContainer.led1.setStatus(LEDStatus.targetSearching);
				limeLightCanSeeTag = false;
			}
		}

		if(isSim) {

			odometry.update(
				Rotation2d.fromDegrees(getHeading()),
				swervePosition);

			poseEstimator.update(
				Rotation2d.fromDegrees(getHeading()),
				swervePosition
			);

		} else {
			odometry.update(
				gyro.getRotation2d().unaryMinus(),
				swervePosition
			);

			estimatedPose = poseEstimator.update(
				gyro.getRotation2d().unaryMinus(),
				swervePosition
			);
		}

		// Show the estimated position
		//Logger.recordOutput("Estimator/Robot", poseEstimator.getEstimatedPosition());
		Logger.recordOutput("Odometry/Robot", odometry.getPoseMeters());

		// Update the field with the location of the robot
		//RobotContainer.field.setRobotPose(odometry.getPoseMeters());
		RobotContainer.field.setRobotPose(poseEstimator.getEstimatedPosition());

		// this needs to be fixed to show if we are in the area of the selected auto position
		/*if(setupAuto && startPosition != null) {
			RobotContainer.led1.setStatus(LEDStatus.problem);
		} else if(setupAuto && startPosition == null) {
			RobotContainer.led1.setStatus(LEDStatus.problem);
		}*/

		if(photonVisionCanSeeTag || limeLightCanSeeTag) {
			RobotContainer.led1.setStatus(LEDStatus.targetAquired);
		} else {
			RobotContainer.led1.setStatus(LEDStatus.problem);
		}
	}

	public void resetEncoders() {
		frontLeft.resetEncoders();
		rearLeft.resetEncoders();
		frontRight.resetEncoders();
		rearRight.resetEncoders();
	}

	public void zeroHeading() {
		gyro.reset();
	}

	public void setHeading(double heading) {
		System.out.println("setHeading called");
		//gyro.setYaw(heading);
	}

	public Command toggleFieldCentric() {
		return runOnce(() -> {
			fieldRelative = !fieldRelative;
		});
	}

	public void setFieldCentric(boolean fieldCentric) {
		fieldRelative = fieldCentric;
	}

	public void stopMotors() {
		frontLeft.stopMotors();
		frontRight.stopMotors();
		rearLeft.stopMotors();
		rearRight.stopMotors();
	}

	public Trajectory.State generateTrajectory(Pose2d targetPose) {

		// 2018 cross scale auto waypoints.
		var sideStart = new Pose2d(
			odometry.getPoseMeters().getX(), 
			odometry.getPoseMeters().getY(),
			odometry.getPoseMeters().getRotation()
		);
		
		var interiorWaypoints = new ArrayList<Translation2d>();
	
		TrajectoryConfig config = new TrajectoryConfig(
			Units.feetToMeters(12),
			Units.feetToMeters(12)
		);

		//config.setReversed(true);

		try {
	
			trajectory = TrajectoryGenerator.generateTrajectory(
				sideStart,
				interiorWaypoints,
				//crossScale,
				targetPose,
				config
			);
		} catch (Exception e) {
			//System.out.println("DriveSubsystem::generateTrajectory() - " + e.getMessage());
			return null;
		}

		return trajectory.sample(trajectory.getTotalTimeSeconds());
	}

	/*public void goToPose(Constants.PoseDefinitions.kFieldPoses targetPose) {

		Pose2d pose = null;

		if(targetPose == Constants.PoseDefinitions.kFieldPoses.AMPLIFIER) {
			if (DriverStation.getAlliance().get() == Alliance.Blue) {
				pose = Constants.PoseDefinitions.kAmplifierPoseBlue;
			} else {
				pose = Constants.PoseDefinitions.kAmplifierPoseRed;
			}
		} else if(targetPose == Constants.PoseDefinitions.kFieldPoses.SOURCE) {
			if (DriverStation.getAlliance().get() == Alliance.Blue) {
				pose = Constants.PoseDefinitions.kSourcePoseBlue;
			} else {
				pose = Constants.PoseDefinitions.kSourcePoseRed;
			}
		}


		goal = generateTrajectory(pose);

		if(goal == null) {
			return;
		}

		// Get the adjusted speeds. Here, we want the robot to be facing
		// 180 degrees (in the field-relative coordinate system).
		ChassisSpeeds adjustedSpeeds = holonomicDriveController.calculate(
			getPoseEstimatorPose2d(),
			goal,
			pose.getRotation()
		);

		SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(adjustedSpeeds);

		setModuleStates(moduleStates);
	}*/

	public boolean getAutoPositionStatusX() {
		return autoPositionStatusX;
	}

	public boolean getAutoPositionStatusY() {
		return autoPositionStatusY;
	}

	public String getAlliance() {
		String alliance = "";
		if(DriverStation.getAlliance().isPresent()) {
			if (DriverStation.getAlliance().get() == Alliance.Blue) {
				alliance = "Blue";
			} else {
				alliance = "Red";
			}
		}

		return alliance;
	}

	public double getAutoX_Position() {
		return autoX_Position;
	}

	public double getAutoY_Position() {
		return autoY_Position;
	}

	public void setMode(kDriveModes mode) {
		this.mode = mode;
	}

	public kDriveModes getMode() {
		return this.mode;
	}

	public boolean autoAim() {
		if(this.mode == kDriveModes.AIM) {
			return true;
		}

		return false;
	}

	public boolean getTargetLocked() {
		return targetLocked;
	}
}
