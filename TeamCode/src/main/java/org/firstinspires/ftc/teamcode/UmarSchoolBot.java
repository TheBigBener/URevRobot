package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class UmarSchoolBot extends LinearOpMode {
    private DcMotor linearMotor;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private Servo clawServo;
    double leftPower;
    double rightPower;
    double linearMotor;
    private ElapsedTime clawTimer = new ElapsedTime();
    boolean clawDown = false;
    double speedMultiplier =  -0.6;
    @Override
    public void runOpMode() {
        ColorSensor colorSensor;
        // Motor config
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class,"rightDrive");
        linearMotor = hardwareMap.get(DcMotor.class, "linearMotor");
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        // Motor directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        linearMotor.setDirection(DcMotor.Direction.FORWARD);
        
        clawServo.setPosition(0.0);
        
        final int CYCLE_MS = 50;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            speedMultiplier = -0.6;
            if (gamepad1.left_trigger > 0.5) {
                speedMultiplier *= 0.4; // Original - prev was 0.5
                leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            linearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (gamepad2.right_bumper) {
                linearMotor.setPower(-1.0); // Reverse if right bumper pressed
            } else if (gamepad2.right_trigger > 0) {
                linearMotor.setPower(Math.abs(gamepad2.right_trigger)); // Forward with right trigger
            } else {
                linearMotor.setPower(0.0); // Stop linear motor if no input
            }

            if (gamepad2.a && !clawDown && clawTimer.seconds() > 0.6) {
                clawServo.setPosition(0.4);
                clawDown = true;
                clawTimer.reset();
            } else if (gamepad2.a && clawDown && clawTimer.seconds() > 0.6) {
                clawServo.setPosition(0.0);
                clawTimer.reset();
                clawDown = false;
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

