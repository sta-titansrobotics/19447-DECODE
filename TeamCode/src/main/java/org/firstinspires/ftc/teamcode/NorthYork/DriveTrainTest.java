package org.firstinspires.ftc.teamcode.NorthYork;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NorthYork.mechanisms.DriveTrain;

@TeleOp
public class DriveTrainTest extends LinearOpMode {

    DriveTrain driveTrain = new DriveTrain();

    @Override
    public void runOpMode() throws InterruptedException{
        driveTrain.init(hardwareMap);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){
            driveTrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.back, gamepad1.right_bumper);
        }
    }
}
