// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.IO_VisionBase.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class SUB_Vision extends SubsystemBase {
	private final VisionConsumer consumer;
	private final IO_VisionBase[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;

	public SUB_Vision(VisionConsumer consumer, IO_VisionBase... io) {
		this.consumer = consumer;
		this.io = io;

		// Initialize inputs
		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
		}

		// Initialize disconnected alerts
		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++) {
			disconnectedAlerts[i] =
					new Alert(
							"Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
		}
	}

	/**
	 * Returns the X angle to the best target, which can be used for simple servoing with vision.
	 *
	 * @param cameraIndex The index of the camera to use.
	 */
	public Rotation2d getTargetX(int cameraIndex) {
		return inputs[cameraIndex].latestTargetObservation.tx();
	}

	@Override
	public void periodic() {
		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
			Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
		}

		// Initialize logging values
		List<Pose3d> allTagPoses = new LinkedList<>();
		List<Pose3d> allRobotPoses = new LinkedList<>();
		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
		List<Pose3d> allRobotPosesRejected = new LinkedList<>();
		List<Integer> allTagsUsed = new LinkedList<>(); // Track all tags used

		// Define the set of allowed tag IDs
		Set<Integer> allowedTagIds = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			// Initialize logging values
			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();
			List<Integer> tagsUsed = new LinkedList<>(); // Track tags used by this camera

			// Add tag poses - only for allowed tags
			for (int tagId : inputs[cameraIndex].tagIds) {
				if (allowedTagIds.contains(tagId)) {
					var tagPose = aprilTagLayout.getTagPose(tagId);
					if (tagPose.isPresent()) {
						tagPoses.add(tagPose.get());
						tagsUsed.add(tagId); // Add to list of tags used by this camera
						allTagsUsed.add(tagId); // Add to list of all tags used
					}
				}
			}

			// Loop over pose observations
			for (var observation : inputs[cameraIndex].poseObservations) {
				// Keep track of which tags were used in this observation
				List<Integer> observationTags = new LinkedList<>();

				// Check if this observation contains any allowed tags
				boolean containsAllowedTag = false;
				for (int tagId : inputs[cameraIndex].tagIds) {
					if (allowedTagIds.contains(tagId)) {
						containsAllowedTag = true;
						observationTags.add(tagId);
					}
				}

				// Skip this observation entirely if it doesn't contain any allowed tags
				if (!containsAllowedTag) {
					robotPoses.add(observation.pose());
					robotPosesRejected.add(observation.pose());
					continue;
				}

				// Check whether to reject pose
				boolean rejectPose =
						observation.tagCount() == 0 // Must have at least one tag
								|| (observation.tagCount() == 1
										&& observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
								|| Math.abs(observation.pose().getZ())
										> maxZError // Must have realistic Z coordinate

								// Must be within the field boundaries
								|| observation.pose().getX() < 0.0
								|| observation.pose().getX() > aprilTagLayout.getFieldLength()
								|| observation.pose().getY() < 0.0
								|| observation.pose().getY() > aprilTagLayout.getFieldWidth();

				// Add pose to log
				robotPoses.add(observation.pose());
				if (rejectPose) {
					robotPosesRejected.add(observation.pose());
				} else {
					robotPosesAccepted.add(observation.pose());
					// Log the tags used in this accepted observation
					int[] observationTagsArray =
							observationTags.stream().mapToInt(Integer::intValue).toArray();
					Logger.recordOutput(
							"Vision/Camera"
									+ Integer.toString(cameraIndex)
									+ "/Observation"
									+ robotPosesAccepted.size()
									+ "/TagsUsed",
							observationTagsArray);
				}

				// Skip if rejected
				if (rejectPose) {
					continue;
				}

				// Calculate standard deviations
				double stdDevFactor =
						Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
				double linearStdDev = linearStdDevBaseline * stdDevFactor;
				double angularStdDev = angularStdDevBaseline * stdDevFactor;
				if (observation.type() == PoseObservationType.MEGATAG_2) {
					linearStdDev *= linearStdDevMegatag2Factor;
					angularStdDev *= angularStdDevMegatag2Factor;
				}
				if (cameraIndex < cameraStdDevFactors.length) {
					linearStdDev *= cameraStdDevFactors[cameraIndex];
					angularStdDev *= cameraStdDevFactors[cameraIndex];
				}

				// Send vision observation
				consumer.accept(
						observation.pose().toPose2d(),
						observation.timestamp(),
						VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			}

			// Log camera data
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
					tagPoses.toArray(new Pose3d[tagPoses.size()]));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
					robotPoses.toArray(new Pose3d[robotPoses.size()]));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
					robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
					robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

			// Convert to primitive int array for Logger
			int[] tagsUsedArray = tagsUsed.stream().mapToInt(Integer::intValue).toArray();
			Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/TagsUsed", tagsUsedArray);

			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}

		// Log summary data
		Logger.recordOutput(
				"Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPosesAccepted",
				allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPosesRejected",
				allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

		// Convert to primitive int array for Logger
		int[] allTagsUsedArray = allTagsUsed.stream().mapToInt(Integer::intValue).toArray();
		Logger.recordOutput("Vision/Summary/TagsUsed", allTagsUsedArray);
	}

	@FunctionalInterface
	public static interface VisionConsumer {
		public void accept(
				Pose2d visionRobotPoseMeters,
				double timestampSeconds,
				Matrix<N3, N1> visionMeasurementStdDevs);
	}
}
