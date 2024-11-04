package frc.robot.Tools;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight {
    private PoseEstimate limelightMeasurement = null;
    private boolean isSim = false;

    public Limelight() {
        
    }

    public PoseEstimate getResult() {
        if (Constants.kEnableLimelight) {

			if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
				limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");	
			} else {
				limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");	
			}

            return limelightMeasurement;

			/*if(limelightMeasurement != null) {
				if(limelightMeasurement.tagCount >= 2) {
     				//poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
     				poseEstimator.addVisionMeasurement(
         				limelightMeasurement.pose,
         				limelightMeasurement.timestampSeconds
					);
   				}
			}*/
		}

        return null;
    }
}
