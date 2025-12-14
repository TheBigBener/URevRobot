package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
public class UmarSchoolBot extends LinearOpMode {
    private DcMotor intakeMotor;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    double leftPower;
    double rightPower;
    double linearMotor;
    double speedMultiplier = 1.0;
    @Override
    public void runOpMode() {
        ColorSensor colorSensor;
        // Motor config
        DcMotorEx leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        DcMotorEx rightDrive = hardwareMap.get(DcMotorEx.class,"rightDrive");
        DcMotorEx linearMotor = hardwareMap.get(DcMotorEx.class, "linearMotor");
        // Motor directions
        leftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        linearMotor.setDirection(DcMotorEx.Direction.FORWARD);

        final int CYCLE_MS = 50;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            speedMultiplier = 1.0;
            if (gamepad1.left_trigger > 0.5) {
                speedMultiplier *= 0.4; // Original - prev was 0.5
                leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            } else {
                leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }
            linearMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            if (gamepad2.right_bumper) {
                linearMotor.setPower(-1.0); // Reverse if right bumper pressed
            } else if (gamepad2.right_trigger > 0) {
                linearMotor.setPower(Math.abs(gamepad2.right_trigger)); // Forward with right trigger
            } else {
                linearMotor.setPower(0.0); // Stop linear motor if no input
            }

            arcadeDrive(-gamepad1.right_stick_x, -gamepad1.left_stick_y);

            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }
    }

    void arcadeDrive(double forward, double rotate) {
        rightPower = forward + rotate;
        leftPower = forward - rotate;
        leftDrive.setPower(leftPower* speedMultiplier);
        rightDrive.setPower(rightPower * speedMultiplier);


    }
}

