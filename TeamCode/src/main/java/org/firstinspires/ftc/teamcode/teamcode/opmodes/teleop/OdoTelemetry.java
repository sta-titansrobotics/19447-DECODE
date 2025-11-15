package com.teamcode.opmodes.teleop;

import static com.teamcode.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.teamcode.subsystems.Odometry;
import com.teamcode.util.Pose2d;

@TeleOp(name = "OdoTelemetry", group = "Test")
public class OdoTelemetry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odo = new Odometry(hardwareMap);

        VoltageSensor vs = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

        telemetry.setMsTransmissionInterval(50);
        waitForStart();

        long lastNs = System.nanoTime();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = (now - lastNs) / 1e9;
            lastNs = now;

            odo.update();
            Pose2d p = odo.getPose();

            telemetry.addLine("ODOMETRY");
            telemetry.addData("x (in)", "%.3f", p.x);
            telemetry.addData("y (in)", "%.3f", p.y);
            telemetry.addData("hdg (deg)", "%.2f", Math.toDegrees(p.heading));
            telemetry.addData("vx (in/s)", "%.2f", odo.getVx());
            telemetry.addData("vy (in/s)", "%.2f", odo.getVy());
            telemetry.addData("w (deg/s)", "%.1f", Math.toDegrees(odo.getOmega()));
            telemetry.addData("dt (ms)", "%.1f", dt * 1000.0);
            if (vs != null) telemetry.addData("battery (V)", "%.2f", vs.getVoltage());
            if (TELEMETRY_VERBOSE) telemetry.addData("imu", USE_IMU ? "on" : "off");

            telemetry.update();
            idle();
        }
    }
}
