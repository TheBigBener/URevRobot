package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="DecodeAuto", group="Decode")
public  class AutoExample extends OpMode {
    private DcMotorEx launcher;
    double launcher_power = 1.0;
    // launcher velocities (tune to your hardware)
    final double LAUNCHER_TARGET_VELOCITY = 1850.0;
    final double LAUNCHER_MIN_VELOCITY = 1750.0;

    double shotsToFire = 3;
    double TIME_BETWEEN_SHOTS = 1.0;    // reduced cycle time (tune)
    double boxServoTime = 0.5;          // servo dwell time (tune)

    boolean driveOffLine = true;

    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime boxServoTimer = new ElapsedTime();
    private ElapsedTime launcherSpinupTimer = new ElapsedTime();

    // motion constants (tune these distances to match your robot and field)
    final double DRIVE_SPEED = 0.5;
    double robotRotationAngle = -45.0;
    final double ROTATE_SPEED = 1.0;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    // Distances (in inches) — TUNE these on your field:
    // Distance from starting position (front of red basket) to the shoot spot (edge where lines meet)
    final double SHOOT_POSITION_DISTANCE_IN = 48.0;      // <-- tune this to place robot at the meeting point of lines
    // Distance to drive BACK into the middle of the field after shooting
    final double RETURN_TO_MIDDLE_DISTANCE_IN = 24.0;    // <-- tune to move into middle of field

    private DcMotorEx frontLeft = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backRight = null;
    private DcMotor intakeMotor = null;
    private Servo boxServo = null;

    // launch state machine
    private enum LaunchState { IDLE, PREPARE, LAUNCH }
    private LaunchState launchState;

    // autonomous high-level states
    private enum AutonomousState {
        DRIVE_TO_SHOOT,
        LAUNCH,
        WAIT_FOR_LAUNCH,
        ROTATING,
        RETURN_TO_MIDDLE,
        COMPLETE
    }
    private AutonomousState autonomousState;

    // flags to ensure we set run-to-position targets only once when entering a state
    private boolean driveTargetSet = false;
    private boolean rotateTargetSet = false;

    @Override
    public void init() {
        autonomousState = AutonomousState.DRIVE_TO_SHOOT;
        launchState = LaunchState.IDLE;

        // Hardware mapping
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        boxServo = hardwareMap.get(Servo.class, "boxServo");

        // Motor directions
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotorEx.Direction.FORWARD);

        // Reset encoders & braking
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        intakeMotor.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300,0,0,10));

        // initial servo position (closed)
        boxServo.setPosition(0.85);
        telemetry.addData("Init", "Complete");
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) {
            driveOffLine = false;
        } else {
            driveOffLine = true;
        }
        telemetry.addData("Press X", " to not drive off the line!");
        telemetry.addData("Drive off line: ", driveOffLine);
        telemetry.addData("Aiden", " sucks >:(");
    }

    @Override
    public void start() {
        // reset shot counter each match start
        shotsToFire = 3;
    }

    @Override
    public void loop() {
        telemetry.addData("State", autonomousState);
        telemetry.addData("LaunchState", launchState);
        switch (autonomousState) {
            case DRIVE_TO_SHOOT:
                // Drive forward from start to shooting spot (distance positive = forward)
                if (drive(DRIVE_SPEED, SHOOT_POSITION_DISTANCE_IN, DistanceUnit.INCH, 0.5)) {
                    // reached shoot position; reset drive flags for next movement
                    resetDriveFlags();
                    // prepare launcher sequence
                    autonomousState = AutonomousState.LAUNCH;
                }
                break;

            case LAUNCH:
                // start shot
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                if (launch(false)) {
                    shotsToFire -= 1;
                    if (shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        // finished firing all shots; stop launcher and drive back to middle
                        launcher.setVelocity(0);
                        if (driveOffLine) {
                            autonomousState = AutonomousState.ROTATING;
                        } else {
                            autonomousState = AutonomousState.COMPLETE;
                        }
                        resetDriveFlags();
                    }
                }
                break;

            case RETURN_TO_MIDDLE:
                // Drive BACK toward middle of field; here we use a positive distance to drive forward
                // because our drive() interprets "distance" direction consistently per call.
                if (drive(DRIVE_SPEED, -RETURN_TO_MIDDLE_DISTANCE_IN, DistanceUnit.INCH, 0.5)) {
                    resetDriveFlags();
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
            case ROTATING:
                if(rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES,1)){
                    frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    resetDriveFlags();
                    if (driveOffLine) {
                        autonomousState = AutonomousState.RETURN_TO_MIDDLE;
                    } else {
                        autonomousState = AutonomousState.COMPLETE;
                    }
                }
                break;

            case COMPLETE:
                // stop all motion
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
                intakeMotor.setPower(0);
                launcher.setVelocity(0);
                // nothing else to do
                break;
        }
    }

    // reset flags used for setting targets once per state
    private void resetDriveFlags() {
        driveTargetSet = false;
        rotateTargetSet = false;
        driveTimer.reset();
    }

    // Launch routine: request shotRequested=true one time to start a shot
    boolean launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    launcherSpinupTimer.reset();
                    shotTimer.reset();
                }
                break;

            case PREPARE:
                // Spin up launcher
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                // Wait for either sufficient velocity OR a short timeout (failsafe)
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY || launcherSpinupTimer.seconds() > 1.5) {
                    launchState = LaunchState.LAUNCH;
                    intakeMotor.setPower(1.0);
                    boxServo.setPosition(0.6); // open box to feed
                    boxServoTimer.reset();
                    shotTimer.reset();
                }
                break;

            case LAUNCH:
                // Wait for servo to move and ball to feed
                if (boxServoTimer.seconds() > boxServoTime) {
                    // stop intake and close box briefly
                    intakeMotor.setPower(0.0);
                    boxServo.setPosition(0.85);

                    // wait between shots
                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        launchState = LaunchState.IDLE;
                        return true; // signal shot finished
                    }
                }
                break;
        }
        return false;
    }

    // Drive: improved to set targets only once per entry to state
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        if (!driveTargetSet) {
            // reset encoders so target is relative to current pose
            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setTargetPosition((int) targetPosition);
            backLeft.setTargetPosition((int) targetPosition);
            frontRight.setTargetPosition((int) targetPosition);
            backRight.setTargetPosition((int) targetPosition);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            driveTimer.reset();
            driveTargetSet = true;
        }

        // If not at target, reset hold timer
        if (Math.abs(targetPosition - frontLeft.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * rotate: simple encoder-based rotate (keeps existing implementation,
     * but not used in this version; leave it for future tuning)
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);
        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        if (!rotateTargetSet) {
            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setTargetPosition((int) leftTargetPosition);
            backLeft.setTargetPosition((int) leftTargetPosition);
            frontRight.setTargetPosition((int) rightTargetPosition);
            backRight.setTargetPosition((int) rightTargetPosition);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            driveTimer.reset();
            rotateTargetSet = true;
        }

        if ((Math.abs(leftTargetPosition - frontLeft.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}

