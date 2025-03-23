// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public class PoseAllignment {

	public final List<Pose2d> blueLeft = new ArrayList<>();
	public final List<Pose2d> blueRight = new ArrayList<>();
	public final List<Pose2d> redLeft = new ArrayList<>();
	public final List<Pose2d> redRight = new ArrayList<>();
	public final List<Pose2d> HPRed = new ArrayList<>();
	public final List<Pose2d> HPBlue = new ArrayList<>();

	public PoseAllignment() {
		blueLeft.add(new Pose2d(3.190, 4.142, new Rotation2d()));
		blueLeft.add(new Pose2d(3.652, 2.929, new Rotation2d(1.000)));
		blueLeft.add(new Pose2d(4.981, 2.764, new Rotation2d(2.127)));
		blueLeft.add(new Pose2d(5.831, 3.858, new Rotation2d(-3.141)));
		blueLeft.add(new Pose2d(5.344, 5.112, new Rotation2d(-2.121)));
		blueLeft.add(new Pose2d(4.011, 5.271, new Rotation2d(-1.099)));

		blueRight.add(new Pose2d(3.193, 3.851, new Rotation2d()));
		blueRight.add(new Pose2d(3.878, 2.734, new Rotation2d(1.000)));
		blueRight.add(new Pose2d(5.262, 2.929, new Rotation2d(2.127)));
		blueRight.add(new Pose2d(5.842, 4.100, new Rotation2d(-3.141)));
		blueRight.add(new Pose2d(5.042, 5.287, new Rotation2d(-2.121)));
		blueRight.add(new Pose2d(3.673, 5.130, new Rotation2d(-1.099)));

		redLeft.add(new Pose2d(11.592, 4.175, new Rotation2d()));
		redLeft.add(new Pose2d(12.24, 2.95, new Rotation2d(0.980)));
		redLeft.add(new Pose2d(13.60, 2.78, new Rotation2d(2.133)));
		redLeft.add(new Pose2d(14.541, 3.863, new Rotation2d(-3.188)));
		redLeft.add(new Pose2d(12.258, 5.077, new Rotation2d(-2.119)));
		redLeft.add(new Pose2d(12.476, 5.224, new Rotation2d(-1.028)));

		redRight.add(new Pose2d(11.532, 3.887, new Rotation2d()));
		redRight.add(new Pose2d(12.58, 2.73, new Rotation2d(0.980)));
		redRight.add(new Pose2d(13.930, 2.808, new Rotation2d(2.133)));
		redRight.add(new Pose2d(14.517, 4.163, new Rotation2d(-3.188)));
		redRight.add(new Pose2d(13.678, 5.338, new Rotation2d(-2.119)));
		redRight.add(new Pose2d(12.203, 5.266, new Rotation2d(-1.028)));

		HPRed.add(new Pose2d(16.416, 7.096, new Rotation2d(-2.202)));
		HPRed.add(new Pose2d(16.234, 0.871, new Rotation2d(2.121)));

		HPBlue.add(new Pose2d(1.182, 7.060, new Rotation2d(-0.945)));
		HPBlue.add(new Pose2d(1.085, 0.955, new Rotation2d(0.955)));
	}
}
