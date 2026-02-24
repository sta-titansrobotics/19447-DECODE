package org.firstinspires.ftc.teamcode.NorthYork;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.NorthYork.mechanisms.DriveTrain;

@TeleOp
public class TeleOpNutsDrop extends LinearOpMode {
    DriveTrain driveTrain = new DriveTrain();
    private DcMotor intakeMotor, indexMotor;
    private DcMotorEx shootMotor;

    private CRServo leftRoller, rightRoller;
    private double targetVelocity;


    @Override
    public void runOpMode() throws InterruptedException{
        driveTrain.init(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        indexMotor = hardwareMap.get(DcMotor.class, "indexMotor");

        leftRoller = hardwareMap.get(CRServo.class, "leftRoller");
        rightRoller = hardwareMap.get(CRServo.class,"rightRoller");

        shootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        targetVelocity = 0;

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){
            if(gamepad2.right_bumper){
                targetVelocity += 100;
                sleep(100);
            }else if (gamepad2.left_bumper){
                targetVelocity -= 100;
                sleep(100);
            }else if(gamepad2.x){
                targetVelocity = 0;
            }
            if(targetVelocity < 0){
                targetVelocity = 0;
            }
            shootMotor.setVelocity(targetVelocity);
            telemetry.addData("Velocity", shootMotor.getVelocity());
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.update();

            driveTrain.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.back, gamepad1.right_bumper);
        }
    }
}
