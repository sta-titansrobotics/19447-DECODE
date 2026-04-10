package org.firstinspires.ftc.teamcode.NorthYork;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NorthYork.mechanisms.Camera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AprilTagTest extends OpMode {
    Camera camera = new Camera();

    @Override
    public void init() {
        camera.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        camera.update();

        AprilTagDetection id20 = camera.getTagBySpecificId(20);
        camera.displayDetectionTelemetry(id20);
        //telemetry.addData("id20 String", id20.toString());
     }

}
