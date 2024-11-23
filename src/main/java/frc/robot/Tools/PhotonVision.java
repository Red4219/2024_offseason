package frc.robot.Tools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Vector;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.PhotonVisionConstants;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.RobotBase;

//import edu.wpi.first.cscore.VideoMode.PixelFormat;

public class PhotonVision {
	private PhotonCamera _camera;
	//private PhotonCamera _camera = null;
	
	// simulation variables
	//private SimVisionSystem _simVisionSystem;
	private VisionSystemSim _visionSystemSim;
	private Pose3d _sim_farTargetPose;
	private double _sim_targetWidth;
	private double _sim_targetHeight;
	private AprilTagFieldLayout _aprilTagFieldLayout;
	private PhotonPoseEstimator _photonPoseEstimator;
	ShuffleboardTab photonVisionTab;
	private EstimatedRobotPose _estimatedRobotPose;
	private int[] _targetsUsed = new int[0];
	private int _speakerTarget = 0;
	PhotonPipelineResult result;
	List<PhotonTrackedTarget> targets;
	private Pose2d prevEstimatedRobotPose = null;
	private PhotonVisionResult photonVisionResult = new PhotonVisionResult(false, new Pose2d(), 0.0);
	private Pose2d prevPhotonEstimatedPose = null;
	List<Pose3d> allTagPoses = new ArrayList<>();
	Optional<EstimatedRobotPose> estimatedRobotPose = null;

	Pose3d camPose = new Pose3d();
	private Pose2d _lastPhotonPoseEstimatorPose = new Pose2d();

	private boolean isSim = false;
	
	public PhotonVision() {

		// Is PhotonVision Enabled?
		if (Constants.kEnablePhotonVision) {

			// Is this a simulation?
			if (RobotBase.isReal()) {
				isSim = false;
			} else {
				isSim = true;
			}

			_camera = new PhotonCamera(PhotonVisionConstants.CameraName);

			try {
				// This sets up the field with the correct Apriltags in the proper places
				// These values are used later for aiming and localization
				_aprilTagFieldLayout = AprilTagFieldLayout
						.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

				// Set if we are blue or red
				// Set the origin accordingly
				if (DriverStation.getAlliance().isPresent()) {

					if (DriverStation.getAlliance().get() == Alliance.Blue) {
						_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
					} else {
						_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
					}
				} else {
					_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
				}

			} catch (IOException e) {
				System.out.println("PhotonVision::PhotonVision() - error:" + e.toString());
				return;
			}

			// If we are in a simulator, set it
			if(isSim) {
				System.out.println("running setupSimulation()");
				setupSimulation(new Pose3d());
			}

			// If the field layout is created and valid and the camera is created and connected
			// create the photon pose estimator.  It is where photonvision thinks the robot is.
			if (_aprilTagFieldLayout != null) {
				if (_camera != null) {
					if (_camera.isConnected()) {
						_photonPoseEstimator = new PhotonPoseEstimator(
								_aprilTagFieldLayout,
								Constants.PhotonVisionConstants.poseStrategy,
								_camera,
								Constants.PhotonVisionConstants.cameraToRobot);
					} else {
						System.out.println("PhotonVision::PhotonVision() - the camera is not connected");
					}
				} else {
					System.out.println("PhotonVision::PhotonVision() - _camera is null");
				}
			}

			// Create the elements in Shuffleboard for debugging if debugPhotonVision is true 
			if (Constants.debugPhotonVision == true) {
				photonVisionTab = Shuffleboard.getTab("PhotonVision");

				photonVisionTab.addDouble("Target Distance", this::getTargetDistance);
				photonVisionTab.addBoolean("Connection", this::isConnected);
				photonVisionTab.addBoolean("Has Target", this::hasTarget);
				photonVisionTab.addString("Targets Used", this::targetsUsed);
				// photonVisionTab.addDouble("Target Used", this::getTargetUsed);
				photonVisionTab.addDouble("Speaker ID", this::getSpeakerTarget);
			}

			// Start a new thread that runs faster ~10 milliseconds instead of ~20 milliseconds
			Thread thread = new Thread() {
				public void run() {

					while (true) {

						if (isSim) {
							// Update PhotonVision based on our new robot position.
							//_simVisionSystem.processFrame(prevEstimatedRobotPose);

							if (prevEstimatedRobotPose == null) {
								prevEstimatedRobotPose = new Pose2d();
							}

							_visionSystemSim.update(prevEstimatedRobotPose);
						}

						if (prevEstimatedRobotPose == null) {
							System.out.println("PhonVision::getPose() - prevEstimatedRobotPose is null");
							prevEstimatedRobotPose = new Pose2d();
						}

						try {
							result = _camera.getLatestResult();
							targets = result.getTargets();
							_targetsUsed = new int[targets.size()];

							// List<Pose3d> allTagPoses = new ArrayList<>();
							allTagPoses.clear();

							int i = 0;
							for (PhotonTrackedTarget target : targets) {
								allTagPoses.add(
										_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get());

								_targetsUsed[i] = target.getFiducialId();
								i++;
							}

							///////////////

							Logger.recordOutput(
									"AprilTagVision/TargetsUsed",
									allTagPoses.toArray(new Pose3d[allTagPoses.size()]));

							/////////

							Optional<EstimatedRobotPose> o = getPhotonPose(prevEstimatedRobotPose);

							if (o.isPresent()) {

								// System.out.println("PhonVision::getPose() - it is present");

								// estimatedRobotPose = o.get();
								// _estimatedRobotPose = estimatedRobotPose;
								_estimatedRobotPose = o.get();

								for (PhotonTrackedTarget target : _estimatedRobotPose.targetsUsed) {
									allTagPoses.add(
											_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get());
								}

								/*
								 * Logger.recordOutput(
								 * "AprilTagVision/TagPoses",
								 * allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
								 */

								// return new PhotonVisionResult(true, estimatedRobotPose.estimatedPose,
								// estimatedRobotPose.timestampSeconds);

								prevPhotonEstimatedPose = _estimatedRobotPose.estimatedPose.toPose2d();

								photonVisionResult
										.setTargetFound(true)
										.setImageCaptureTime(_estimatedRobotPose.timestampSeconds)
										.setPose2d(prevPhotonEstimatedPose);

								// photonVisionResult = new PhotonVisionResult(true,
								// _estimatedRobotPose.estimatedPose,
								// _estimatedRobotPose.timestampSeconds);

								Logger.recordOutput("PhotonVisionEstimator/Robot", prevPhotonEstimatedPose);

							} else {
								// System.out.println("PhonVision::getPose() - I don't see any tags");
								// Since we do not have any tags that we can see, blank out the list
								// System.out.println("we do not have a pose");
								Logger.recordOutput(
										"AprilTagVision/TagPoses",
										allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
							}

							///////////////

							Thread.sleep(5);

						} catch (Exception e) {
							e.printStackTrace();
							System.out.println(e);
						}
					}
				}
			};

			thread.start();
		}
	}

	// Is the camera connected?
	public boolean isConnected() {
		if(Constants.kEnablePhotonVision) {
			if(_camera != null) {
				return _camera.isConnected();
			} else {
				return false;
			}
		} else {
			return false;
		}
	}

	// Get the pose/location on the field that photonvision thinks the robot is at
	public PhotonVisionResult getPose(Pose2d prevEstimatedRobotPose) {
		this.prevEstimatedRobotPose = prevEstimatedRobotPose;

		if(this._estimatedRobotPose == null) {
			return null;
		}		
		return new PhotonVisionResult(true, _estimatedRobotPose.estimatedPose.toPose2d(), _estimatedRobotPose.timestampSeconds);
	}

	// Set the reference 2D (X,Y) pose/position for PhotonVision to use
	public void setReferencePose(Pose2d referencePose) {
		
		if(_photonPoseEstimator != null) {
			_photonPoseEstimator.setReferencePose(referencePose);
		}
	}

	// Set the reference 3D (X,Y,Z) pose/position for PhotonVision to use
	public void setReferencePose(Pose3d referencePose) {
		if(_photonPoseEstimator != null) {
			_photonPoseEstimator.setReferencePose(referencePose);
		}
	}

	// Does Photonvision have a target?
	public boolean hasTarget() {
		try {
			if(_camera.getLatestResult().hasTargets()) {
				return true;
			}
		} catch (Exception e) {
			e.printStackTrace();
			return false;
		}

		return false;
	}

	// How far is the specified target
	public double targetDistance(int targetNumber) {
		PhotonPipelineResult result = _camera.getLatestResult();
		List<PhotonTrackedTarget> targets = result.getTargets();

		for(PhotonTrackedTarget target: targets) {
			if(target.getFiducialId() == targetNumber) {

				double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                Constants.PhotonVisionConstants.camHeightOffGround,
                                _aprilTagFieldLayout.getTagPose(targetNumber).get().getZ(),
								//Constants.PhotonVisionConstants.TagHeight,
                                Constants.PhotonVisionConstants.camPitch,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

				return range;
			}
		}

		return 0.0;
	}

	public double getTargetDistance() {
		double distance = 0.0;

		if(_estimatedRobotPose != null) {
			for(PhotonTrackedTarget target : _estimatedRobotPose.targetsUsed) {
				if(_speakerTarget == 0) {
					if(target.getFiducialId() == 7) {
						if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
							distance = targetDistance(7);
							_speakerTarget = 7;
							Logger.recordOutput("AprilTagVision/Target", distance);
							return distance;
						}
					} else if(target.getFiducialId() == 4) {
						if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
							distance = targetDistance(4);
							_speakerTarget = 4;
							Logger.recordOutput("AprilTagVision/Target", distance);
							return distance;
						}
					}
				} else {
					// we have already set the speaker target so just set the distance
					distance = targetDistance(_speakerTarget);
					Logger.recordOutput("AprilTagVision/Target", distance);
					return distance;
				}
			}
		} else {
			//System.out.println("getTargetDistance() - estimatedRobotPose is null");
		}
		
		return distance;
	}

	// Aim at the specified target
	public double aimAtTarget(int targetNumber) {

		PhotonPipelineResult result = _camera.getLatestResult();
		List<PhotonTrackedTarget> targets = result.getTargets();

		for(PhotonTrackedTarget target: targets) {
			if(target.getFiducialId() == targetNumber) {
				return target.getYaw();
			}
		}

		return 0.0;
	}

	// Can photon vision see the specified target?
	public boolean canSeeTarget(int targetNumber) {

		//PhotonTrackedTarget targetToAimAt = null;
		PhotonPipelineResult result = _camera.getLatestResult();

		if(result.hasTargets() == false) {
			//System.out.println("Don't see any targets");
			return false;
		}

		// if we get here, we can see some targets, just might not be the correct one

		List<PhotonTrackedTarget> targets = result.getTargets();

		for(PhotonTrackedTarget target: targets) {
			if(target.getFiducialId() == targetNumber) {
				//System.out.println("target: " + target.getFiducialId());
				return true;
			}
		}

		return false;
	}

	// This is a private method used only internally
	private synchronized Optional<EstimatedRobotPose> getPhotonPose(Pose2d prevEstimatedRobotPose) {

		try {

			if (_photonPoseEstimator != null) {

				// Check if we are in simulation and the previousEstimatedRobotPose is not null
				// and we are not connected to the camera
				// Change this for testing
				if (prevEstimatedRobotPose != null) {
					_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
					// System.out.println("PhotonVision::getPhotonPose() - x: " +
					// prevEstimatedRobotPose.getX() + " y: " + prevEstimatedRobotPose.getY() + "
					// rotation: " + prevEstimatedRobotPose.getRotation().getDegrees());
				} else {
					System.out.println("PhotonVision::getPhotonPose() - prevEstimatedRobotPose is null");
					prevEstimatedRobotPose = new Pose2d();
				}

				// Optional<EstimatedRobotPose> estimatedRobotPose =
				// _photonPoseEstimator.update();
				estimatedRobotPose = _photonPoseEstimator.update();

				// List<Pose3d> allTagPoses = new ArrayList<>();
				allTagPoses.clear();

				if (estimatedRobotPose.isPresent()) {
					_lastPhotonPoseEstimatorPose = estimatedRobotPose.get().estimatedPose.toPose2d();

					try {

						for (PhotonTrackedTarget target : estimatedRobotPose.get().targetsUsed) {
							if (target.getFiducialId() >= 1 && target.getFiducialId() <= 8) {
								allTagPoses.add(_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get());
							}
						}

						Logger.recordOutput(
								"PhotonVisionEstimator/Robot",
								estimatedRobotPose.get().estimatedPose.toPose2d());
					} catch (Exception e) {
						System.out.println(e.toString());
					}
				} else {

					Logger.recordOutput(
							"PhotonVisionEstimator/Robot",
							_lastPhotonPoseEstimatorPose);

					return Optional.empty();
				}

				return estimatedRobotPose;
			} else {
				System.out.println("getPhotonPose() - _photonPoseEstimator is null");

				if (_camera != null) {
					if (_camera.isConnected()) {
						_photonPoseEstimator = new PhotonPoseEstimator(
								_aprilTagFieldLayout,
								Constants.PhotonVisionConstants.poseStrategy,
								_camera,
								Constants.PhotonVisionConstants.cameraToRobot);
					} else {
						System.out.println("-------> the camera is not connected");
					}
				}
			}

			return Optional.empty();
		} catch (Exception e) {
			return Optional.empty();
		}
	}

	// setup the tags and set the origin to how to show the tags
	public void setupAprilTagFieldLayoutSim() {
		//_simVisionSystem.clearVisionTargets();
		_visionSystemSim.clearVisionTargets();

		if(DriverStation.getAlliance().isPresent()) {
			if(DriverStation.getAlliance().get() == Alliance.Blue) {
				_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
			} else {
				_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
			}
		} else {
			_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
		}

		if(_aprilTagFieldLayout != null) {
			_visionSystemSim.addAprilTags(_aprilTagFieldLayout);

			/*_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(1).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				1)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(2).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				2)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(3).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				3)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(4).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				4)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(5).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				5)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(6).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				6)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(7).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				7)
			);

			_simVisionSystem.addSimVisionTarget(new SimVisionTarget(
				_aprilTagFieldLayout.getTagPose(8).get(),
				_sim_targetWidth,
				_sim_targetHeight, 
				8)
			);*/
		}
	}

	private void setupSimulation(Pose3d aprilTagFieldLayoutOrigin) {

		_visionSystemSim = new VisionSystemSim("main");
		_visionSystemSim.addCamera(new PhotonCameraSim(_camera), Constants.PhotonVisionConstants.cameraToRobot);
			
		
		// See
    	// https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    	// page 208
    	_sim_targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
    	// See
    	// https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    	// page 197
    	_sim_targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    	// See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    	// pages 4 and 5
    	//double tgtXPos = Units.feetToMeters(54);
		double tgtXPos = 5;
    	//double tgtYPos =
            //Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
		double tgtYPos = 5;
    	_sim_farTargetPose =
            new Pose3d(
                    new Translation3d(tgtXPos, tgtYPos, 0.5),
                    new Rotation3d(0.0, 0.0, 0.0));		
		
		setupAprilTagFieldLayoutSim();
	}

	public String targetsUsed() {

		if(_targetsUsed.length == 0) {
			return "";
		}

		String targets = "";

		for(int target : _targetsUsed) {
			targets += " " + target;
		}

		return targets;
	}

	public double getTargetUsed() {
		if(this._targetsUsed.length > 0) {
			return this._targetsUsed[0];
		}

		return 0;
	}

	public double getSpeakerTarget() {
		return _speakerTarget;
	}
}
