package frc.robot.subsystems;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Mechanisms.SwerveModule;
import frc.robot.Tools.AutonomousDetail;
import frc.robot.Tools.GyroIONavX;
import frc.robot.Tools.PhotonVision;
import frc.robot.Tools.PhotonVisionResult;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants.kDriveModes;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.RobotBase;

public class DriveSubsystem extends SubsystemBase {

	private boolean fieldRelative = true;
	private boolean gyroTurning = false;
	private double targetRotationDegrees;

	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule rearLeft;
	private final SwerveModule rearRight;

	private SwerveModulePosition[] swervePosition;
	SwerveModuleState[] swerveModuleStates;

	// Initalizing the gyro sensor
	private final AHRS gyro = new AHRS(SPI.Port.kMXP); 
	private final GyroIONavX _gyroIONavX;
	private AutonomousDetail _autoDetailSelected = null;

	private double xSpeed = 0.0;
	private double ySpeed = 0.0;
	private double rot = 0.0;

	private kDriveModes mode = kDriveModes.NORMAL;
	private int speakerTarget = 0;
	private boolean targetLocked = false;
	private boolean isSim = false;


	PhotonVisionResult photonVisionResult = null;

	// Odeometry class for tracking robot pose
	SwerveDriveOdometry odometry;

	// test for auto positioning
	HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
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

	Trajectory trajectory = null;
	Trajectory.State goal = null;

	// PID controller for gyro turning
	private ProfiledPIDController gyroTurnPidController;

	private Field2d field;

	private SwerveDrivePoseEstimator poseEstimator;

	private PhotonVision _photonVision;
	Pose2d photonPose2d;

	double autoX_Position = 0.0;
	double autoY_Position = 0.0;
	boolean autoPositionStatusX = false;
	boolean autoPositionStatusY = false;
	String alliance = "";

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

	private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {

		if(Constants.kEnablePhotonVision) {
			_photonVision = new PhotonVision();
		}

		if(RobotBase.isReal()) {
			isSim = false;
		} else {
			isSim = true;
		}
		
		gyro.reset();
		_gyroIONavX = new GyroIONavX(gyro);

		frontLeft = new SwerveModule(
				"FL",
				ModuleConstants.kFrontLeftDriveMotorPort,
				ModuleConstants.kFrontLeftTurningMotorPort,
				ModuleConstants.kFrontLeftTurningEncoderPort,
				ModuleConstants.kFrontLeftAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains
			);

		frontRight = new SwerveModule(
				"FR",
				ModuleConstants.kFrontRightDriveMotorPort,
				ModuleConstants.kFrontRightTurningMotorPort,
				ModuleConstants.kFrontRightTurningEncoderPort,
				ModuleConstants.kFrontRightAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains
			);

		rearLeft = new SwerveModule(
				"RL",
				ModuleConstants.kRearLeftDriveMotorPort,
				ModuleConstants.kRearLeftTurningMotorPort,
				ModuleConstants.kRearLeftTurningEncoderPort,
				ModuleConstants.kRearLeftAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains
			);

		rearRight = new SwerveModule(
				"RR",
				ModuleConstants.kRearRightDriveMotorPort,
				ModuleConstants.kRearRightTurningMotorPort,
				ModuleConstants.kRearRightTurningEncoderPort,
				ModuleConstants.kRearRightAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains
			);

		swervePosition = new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()
		};

		odometry = new SwerveDriveOdometry(
				DriveConstants.kDriveKinematics,
				gyro.getRotation2d(),
				swervePosition);

		field = new Field2d();

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
				gyro.getRotation2d(),
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
			ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
			swerveTab.addDouble("FL Absolute", frontLeft::getAbsoluteHeading);
			swerveTab.addDouble("FR Absolute", frontRight::getAbsoluteHeading);
			swerveTab.addDouble("RL Absolute", rearLeft::getAbsoluteHeading);
			swerveTab.addDouble("RR Absolute", rearRight::getAbsoluteHeading);
			swerveTab.addDouble("FL Meters", frontLeft::getDistanceMeters);
			swerveTab.addDouble("FR Meters", frontRight::getDistanceMeters);
			swerveTab.addDouble("RL Meters", rearLeft::getDistanceMeters);
			swerveTab.addDouble("RR Meters", rearRight::getDistanceMeters);
			swerveTab.addBoolean("Auto Aim", this::autoAim);
			swerveTab.addBoolean("Target Locked", this::getTargetLocked);
		}
	}

	public PhotonVision getPhotonVision() {
		return _photonVision;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		updateOdometry();

		if(Constants.debugDriveTrain == true) {
			SmartDashboard.putNumber("FL Offset Check", frontLeft.getAbsoluteHeading() + frontLeft.angleZero);
			SmartDashboard.putNumber("FR Offset Check", frontRight.getAbsoluteHeading() + frontRight.angleZero);
			SmartDashboard.putNumber("RL Offset Check", rearLeft.getAbsoluteHeading() + rearLeft.angleZero);
			SmartDashboard.putNumber("RR Offset Check", rearRight.getAbsoluteHeading() + rearRight.angleZero);
			SmartDashboard.putNumber("2D X", getPose().getX());
			SmartDashboard.putNumber("2D Y", getPose().getY());
			SmartDashboard.putNumber("2D Gyro", odometry.getPoseMeters().getRotation().getDegrees());
		}

		SmartDashboard.putData("field", field);
	}

	public void setAutoCommandSelected(Command autoCommand) {
		if(autoCommand != null) {
			_autoDetailSelected = Constants.AutonomousRoutines.get(autoCommand.getName());
			System.out.println("auto selected: " + autoCommand.getName());
		}
	}

	// region getters
	public double getHeading() {
		return gyro.getRotation2d().getDegrees();
	}

	public double getHeading360() {		
		return (gyro.getRotation2d().getDegrees() % 360);
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
			(isSim) ? pose.getRotation() : gyro.getRotation2d(),
			swervePosition,
			pose
		);

		poseEstimator.resetPosition(
			(isSim) ? pose.getRotation() : gyro.getRotation2d(),
			swervePosition,
			pose
		);

		if(Constants.kEnablePhotonVision) {
			_photonVision.setReferencePose(pose);
		}

		Logger.recordOutput("Odometry/Robot", odometry.getPoseMeters());
		Logger.recordOutput("Estimator/Robot", poseEstimator.getEstimatedPosition());
	}
	
	public void lockWheels() {
		double rot = 0;

		swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				new ChassisSpeeds(0, 0, rot));

		SwerveDriveKinematics.desaturateWheelSpeeds(
				swerveModuleStates, 0);

		setModuleStates(swerveModuleStates);
	}

	public void robotCentricDrive(double xSpeed, double ySpeed, double rot) {
		setFieldCentric(false);
		drive(xSpeed, ySpeed, rot);
		setFieldCentric(true);
	}

	public void drive(double xSpeed, double ySpeed, double rot) {
		drive(xSpeed, ySpeed, rot, false, false);
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean isTurbo, boolean isSneak) {

		// Apply deadbands to inputs
		xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
		ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;

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
		if (mode == kDriveModes.AIM) {
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
			//this.lockWheels();
			return;
		}

		swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()));
		
		setModuleStates(swerveModuleStates);
	}

	public ChassisSpeeds getChassisSpeedsRobotRelative() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());
	}

	public void setChassisSpeedsRobotRelative(ChassisSpeeds chassisSpeeds ){

		chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

		SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

		if(isSim) {
			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
			SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			angle.set(angle.get() + -chassisSpeeds.omegaRadiansPerSecond);
		}

		frontLeft.setDesiredState(swerveModuleStates[0]);
		frontRight.setDesiredState(swerveModuleStates[1]);
		rearLeft.setDesiredState(swerveModuleStates[2]);
		rearRight.setDesiredState(swerveModuleStates[3]);

		Logger.recordOutput("SwerveModuleStates/Setpoints", swerveModuleStates);
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

		// For some reason, the code below is preventing rotation in SIM
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
				gyro.getRotation2d(), 
				swervePosition
			);

			poseEstimator.update(
				gyro.getRotation2d(), 
				swervePosition
			);
		}

		if (Constants.kEnablePhotonVision) {
			photonVisionResult = _photonVision.getPose(poseEstimator.getEstimatedPosition());

			if (photonVisionResult.targetFound()) {
				photonPose2d = photonVisionResult.pose2d();

				poseEstimator.addVisionMeasurement(
						photonPose2d,
						Timer.getFPGATimestamp() - photonVisionResult.imageCaptureTime(),
						visionMeasurementStdDevs);
			}
		}

		field.setRobotPose(odometry.getPoseMeters());

		try {
			Logger.recordOutput("Odometry/Robot", odometry.getPoseMeters());
			Logger.recordOutput("Estimator/Robot", poseEstimator.getEstimatedPosition());
		} catch (Exception e) {
			System.out.println(e.toString());
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

	public void goToPose(Constants.PoseDefinitions.kFieldPoses targetPose) {

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
	}

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
