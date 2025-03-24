// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * The AlignmentCamera class captures video from a USB camera, adds alignment rectangle overlays,
 * and publishes the processed video stream to SmartDashboard. Optimized for memory efficiency on
 * the RoboRIO.
 */
public class AlignmentCamera extends SubsystemBase {
	private Thread visionThread;
	private final UsbCamera camera;
	private final CvSink cvSink;
	private final CvSource outputStream;
	// Static objects to avoid repeated creation
	private static final Scalar BLACK_COLOR = new Scalar(0, 0, 0, 0.2);

	private int cameraWidth = 320; // Lower resolution for better performance
	private int cameraHeight = 240;
	private int fps = 30; // Lower framerate to reduce CPU usage

	// Rectangle parameters
	private int leftRectWidth;
	private int rightRectX;

	// Gap in the middle (as percentage of screen width)
	private double gapWidthPct = 0.15;

	private final int rectThickness = -1; // Negative for filled rectangles

	/**
	 * Creates an AlignmentCamera with dual rectangles and a gap in the middle
	 *
	 * @param cameraIndex The USB camera device index
	 * @param name The name for the camera stream
	 */
	public AlignmentCamera(int cameraIndex, String name) {
		this(cameraIndex, name, 0.15);
	}

	/**
	 * Creates an AlignmentCamera with customizable dual rectangles
	 *
	 * @param cameraIndex The USB camera device index
	 * @param name The name for the camera stream
	 * @param gapWidthPct Width of the gap between rectangles (0.0-1.0, as percentage of width)
	 */
	public AlignmentCamera(int cameraIndex, String name, double gapWidthPct) {
		// Initialize the camera with efficient settings
		camera = CameraServer.startAutomaticCapture(cameraIndex);
		camera.setResolution(cameraWidth, cameraHeight);
		camera.setFPS(fps);

		// Optimize camera settings for performance
		camera.setExposureAuto(); // Auto exposure to avoid manual adjustments
		camera.setWhiteBalanceAuto(); // Auto white balance
		camera.setBrightness(50); // Mid brightness to save processing

		// Create sinks and sources for OpenCV processing
		cvSink = CameraServer.getVideo();
		outputStream = CameraServer.putVideo(name + "_alignment", cameraWidth, cameraHeight);

		// Store the gap width and calculate rectangle dimensions
		this.gapWidthPct = gapWidthPct;
		updateRectangleParameters(gapWidthPct);

		// Start the vision processing thread
		startVisionThread();

		// Use lightweight status update
		SmartDashboard.putBoolean(name + "_active", true);
	}

	/**
	 * Updates the alignment rectangles' positions and sizes
	 *
	 * @param gapWidthPct Width of the gap between rectangles (0.0-1.0)
	 */
	public void updateRectangleParameters(double gapWidthPct) {
		this.gapWidthPct = gapWidthPct;

		// Calculate rectangle positions - simplified arithmetic
		int centerX = cameraWidth / 2;
		int halfGap = (int) (cameraWidth * gapWidthPct / 2);

		// Only store the essential values needed for drawing
		this.leftRectWidth = centerX - halfGap;
		this.rightRectX = centerX + halfGap;
	}

	/**
	 * Adjust the width of the gap between rectangles
	 *
	 * @param gapWidthPct Gap width as percentage of screen width (0.0-1.0)
	 */
	public void setGapWidth(double gapWidthPct) {
		updateRectangleParameters(gapWidthPct);
	}

	/**
	 * Starts the vision processing thread that adds the dual rectangles to each frame and sends it to
	 * SmartDashboard. Optimized for memory usage.
	 */
	private void startVisionThread() {
		visionThread =
				new Thread(
						() -> {
							// Create a single Mat object that is reused - important for memory efficiency
							Mat frame = new Mat();
							// Pre-calculate points to avoid creating Point objects in the loop
							Point leftTopLeft = new Point(0, 0);
							Point rightBottomRight = new Point(cameraWidth, cameraHeight);

							try {
								while (!Thread.interrupted()) {
									// Only process if we can grab a frame
									if (cvSink.grabFrame(frame) == 0) {
										// Don't create error strings each time
										outputStream.notifyError(cvSink.getError());
										Thread.sleep(20); // Short sleep on error to prevent CPU spinning
										continue;
									}

									// Use pre-calculated values where possible
									Point leftBottomRight = new Point(leftRectWidth, cameraHeight);
									Point rightTopLeft = new Point(rightRectX, 0);

									// Draw rectangles
									Imgproc.rectangle(
											frame, leftTopLeft, leftBottomRight, BLACK_COLOR, rectThickness);
									Imgproc.rectangle(
											frame, rightTopLeft, rightBottomRight, BLACK_COLOR, rectThickness);

									// Output the frame
									outputStream.putFrame(frame);

									// Small sleep to reduce CPU usage
									Thread.sleep(5);
								}
							} catch (InterruptedException e) {
								// Thread interrupted, clean exit
							} finally {
								// Ensure resources are released
								frame.release();
							}
						});

		visionThread.setDaemon(true);
		visionThread.setPriority(Thread.MIN_PRIORITY); // Lower priority
		visionThread.start();
	}

	/**
	 * Updates the camera resolution and recalculates rectangle positions
	 *
	 * @param width The new width in pixels
	 * @param height The new height in pixels
	 */
	public void setResolution(int width, int height) {
		this.cameraWidth = width;
		this.cameraHeight = height;
		camera.setResolution(width, height);
		updateRectangleParameters(this.gapWidthPct);
	}

	/**
	 * Updates the camera frame rate
	 *
	 * @param fps The new frame rate
	 */
	public void setFPS(int fps) {
		this.fps = fps;
		camera.setFPS(fps);
	}

	@Override
	public void periodic() {
		// Nothing needed here as we're using a separate thread for processing
	}
}
