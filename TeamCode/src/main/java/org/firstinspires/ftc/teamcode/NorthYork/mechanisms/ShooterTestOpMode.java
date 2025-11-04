package org.firstinspires.ftc.teamcode.NorthYork.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class ShooterTestOpMode extends LinearOpMode {

    private DcMotorEx shootMotor;
    private double targetVelocity;

    @Override
    public void runOpMode() throws InterruptedException{

        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        shootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        double targetVelocity = -500;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.dpad_up){
                targetVelocity += 100;
                sleep(100);
            }else if (gamepad1.dpad_down){
                targetVelocity -= 100;
                sleep(100);
            }else if(gamepad1.x){
                targetVelocity = 0;
            }
            if(targetVelocity < 0){
                targetVelocity = 0;
            }
            shootMotor.setVelocity(targetVelocity);
            telemetry.addData("Velocity", shootMotor.getVelocity());
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.update();
        }
    }
}
