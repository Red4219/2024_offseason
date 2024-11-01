package frc.robot.Tools;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class PhotonVisionResult {
	private boolean _targetFound = false;
	private Pose2d _pose;
	private double _imageCaptureTime;

	public PhotonVisionResult(boolean targetFound, Pose2d pose, double imageCaptureTime) {
		_targetFound = targetFound;
		_pose = pose;
		_imageCaptureTime = imageCaptureTime;
	}

	public boolean targetFound() {
		return _targetFound;
	}

	public PhotonVisionResult setTargetFound(boolean targetFound) {
		this._targetFound = targetFound;
		return this;
	}

	public Pose2d pose2d() {
		return _pose;
	}

	public PhotonVisionResult setPose2d(Pose2d pose2d) {
		this._pose = pose2d;
		return this;
	}

	public double imageCaptureTime() {
		return _imageCaptureTime;
	}

	public PhotonVisionResult setImageCaptureTime(double imageCaptureTime) {
		this._imageCaptureTime = imageCaptureTime;
		return this;
	}
}
