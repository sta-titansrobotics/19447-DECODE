package com.teamcode.vision;

import static com.teamcode.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Manages AprilTag detection during INIT and path selection.
 * Maps detected tag IDs to autonomous path variants.
 */
public class AprilTagSelector {

    public enum AutoPath {
        LEFT,    // Tag ID from Constants.APRILTAG_ID_LEFT
        CENTER,  // Tag ID from Constants.APRILTAG_ID_CENTER or default
        RIGHT    // Tag ID from Constants.APRILTAG_ID_RIGHT
    }

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    private int lastSeenTagId = -1;
    private int stableTagId = -1; // For hysteresis
    private ElapsedTime lastSeenTime = new ElapsedTime();
    private AutoPath selectedPath = AutoPath.CENTER; // Default
    private boolean cameraReady = false;

    public AprilTagSelector(WebcamName camera) {
        AprilTagProcessor processor = null;
        VisionPortal portal = null;
        boolean ready = false;

        try {
            // Create AprilTag processor
            processor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .build();

            // Create vision portal
            portal = new VisionPortal.Builder()
                    .setCamera(camera)
                    .addProcessor(processor)
                    .build();

            ready = true;
        } catch (Exception e) {
            // Camera init failed (unplugged, wrong name, etc.)
            processor = null;
            portal = null;
            ready = false;
        }

        aprilTagProcessor = processor;
        visionPortal = portal;
        cameraReady = ready;

        lastSeenTime.reset();
    }

    /**
     * Call this repeatedly during INIT loop to update detections.
     */
    public void update() {
        if (!cameraReady || aprilTagProcessor == null) {
            return; // Camera not available, skip
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections == null || detections.isEmpty()) {
            // No detections this frame
            return;
        }

        // Find best detection (highest decision margin = confidence)
        AprilTagDetection best = null;
        double bestMargin = APRILTAG_MIN_CONFIDENCE;

        for (AprilTagDetection detection : detections) {
            if (detection.decisionMargin > bestMargin) {
                best = detection;
                bestMargin = detection.decisionMargin;
            }
        }

        if (best != null) {
            // Hysteresis: only switch tags if:
            // 1. No stable tag yet (first detection)
            // 2. Same as current stable tag (reinforcement)
            // 3. Significantly better than minimum confidence threshold (prevents flicker)
            boolean shouldSwitch = (stableTagId == -1) ||
                                   (best.id == stableTagId) ||
                                   (bestMargin > APRILTAG_MIN_CONFIDENCE + APRILTAG_TAG_SWITCH_MARGIN);

            if (shouldSwitch) {
                lastSeenTagId = best.id;
                stableTagId = best.id;
                lastSeenTime.reset();

                // Map tag ID to path (using Constants)
                if (lastSeenTagId == APRILTAG_ID_LEFT) {
                    selectedPath = AutoPath.LEFT;
                } else if (lastSeenTagId == APRILTAG_ID_RIGHT) {
                    selectedPath = AutoPath.RIGHT;
                } else {
                    selectedPath = AutoPath.CENTER;
                }
            }
        }
    }

    /**
     * Get the currently selected path based on most recent valid detection.
     * If detection is stale, uses default CENTER path.
     */
    public AutoPath getSelectedPath() {
        if (lastSeenTime.seconds() > APRILTAG_DETECTION_TIMEOUT_S) {
            return AutoPath.CENTER; // Default if no recent detection
        }
        return selectedPath;
    }

    /**
     * @return ID of last seen tag, or -1 if none
     */
    public int getLastSeenTagId() {
        return lastSeenTagId;
    }

    /**
     * @return Age of last detection in seconds
     */
    public double getTimeSinceLastDetection() {
        return lastSeenTime.seconds();
    }

    /**
     * @return True if we have a recent valid detection
     */
    public boolean hasRecentDetection() {
        return lastSeenTime.seconds() < APRILTAG_DETECTION_TIMEOUT_S && lastSeenTagId >= 0;
    }

    /**
     * @return True if camera initialized successfully
     */
    public boolean isCameraReady() {
        return cameraReady;
    }

    /**
     * Stop camera streaming (call after START to save resources).
     */
    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    /**
     * Close vision portal completely.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
