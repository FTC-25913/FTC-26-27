package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Old BioBuzz TeleOp", group="Linear Opmode")
public class OldTeleOp extends LinearOpMode {

    private DcMotorEx nectar, pollen;

    // Declare Drivetrain Motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    // Declare Toggled Auxiliary Motors
    private DcMotor intake = null;

    private static final double NECTAR_TARGET = 1550;  // ticks/sec, tune this
    private static final double VELOCITY_TOLERANCE = 50;
    private static final double POLLEN_TARGET = 1490;

    //servo
    private Servo Nectar_gate;
    private Servo Pollen_gate;

    // Declare Limelight
    private Limelight3A limelight;

    // Toggle logic tracking variables
    private boolean isMotorOn = false;
    private boolean previousXState = false;
    private boolean previousYState = false;
    private boolean IsForward = false;
    private boolean Is_Nectar_Forward = false;
    private boolean Is_Pollen_Forward = false;

    private boolean previous_right_bumper_state = false;
    private boolean previous_right_trigger_state = false;
    private boolean previous_left_bumper_state = false;
    private boolean previous_left_trigger_state = false;
    private boolean isNectar_shooter_on = false;
    private boolean isPollen_shooter_on = false;
    private boolean isNectar_gate_up = false;
    private boolean isPollen_gate_up = false;

    boolean previousAState = false;
    boolean isAOn = false; // Tracks if normal intake is on

    boolean previousBState = false;
    boolean isBOn = false; // Tracks if 100% intake is on

    // Limelight Tuning Constants
    final double TURN_KP = 0.02;
    final double DRIVE_KP = 0.05;
    final double DESIRED_TY = 5.0;

    @Override
    public void runOpMode() {

        // Hardware Mapping - Exact match to your original working chassis configuration
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        //servo
        Nectar_gate = hardwareMap.get(Servo.class, "Nectarservo");
        Pollen_gate = hardwareMap.get(Servo.class, "Pollenservo");

        // Hardware Mapping - Auxiliary toggle motors
        intake     = hardwareMap.get(DcMotor.class, "intake");
        nectar     = hardwareMap.get(DcMotorEx.class, "nectar");
        pollen     = hardwareMap.get(DcMotorEx.class, "pollen");

        // Init (use DcMotorEx instead of DcMotor for these two)
        nectar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nectar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollen.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollen.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Hardware Mapping - Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Motor directions for your chassis orientation
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        //servo

        Nectar_gate.setDirection(Servo.Direction.FORWARD);
        Pollen_gate.setDirection(Servo.Direction.REVERSE);

        // Set auxiliary motors
        intake.setDirection(DcMotor.Direction.REVERSE);
        nectar.setDirection(DcMotor.Direction.FORWARD);
        pollen.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior to BRAKE for crisp stopping
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        nectar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollen.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized (BioBuzz Drivetrain + Aux Motors + Limelight)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =========================================================
            // 1. DRIVETRAIN CONTROLS (Manual vs Auto-Aim)
            // =========================================================
            double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower;

            if (gamepad1.right_bumper) {
                // --- LIMELIGHT AUTO-AIMING ---
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    double tx = result.getTx();
                    double ty = result.getTy();

                    double turnPower = tx * TURN_KP;
                    double drivePower = (DESIRED_TY - ty) * DRIVE_KP;

                    leftFrontPower  = drivePower + turnPower;
                    rightFrontPower = drivePower - turnPower;
                    leftRearPower   = drivePower + turnPower;
                    rightRearPower  = drivePower - turnPower;

                    telemetry.addData("valid", result != null && result.isValid());
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("staleness ms", result.getStaleness());
                    telemetry.addData("pipeline", result.getPipelineIndex());

                    telemetry.addData("Drive Mode", "AUTO-AIMING");
                } else {
                    // Stop if target is lost while holding the button
                    leftFrontPower = rightFrontPower = leftRearPower = rightRearPower = 0;
                    telemetry.addData("Drive Mode", "SEARCHING FOR TARGET");

                }

            } else {
                // --- MANUAL DRIVING ---
                double drive  = -gamepad1.left_stick_y;
                double strafe =  gamepad1.left_stick_x;
                double turn   =  gamepad1.right_stick_x;

                leftFrontPower  = drive + strafe + turn;
                rightFrontPower = drive - strafe - turn;
                leftRearPower   = drive - strafe + turn;
                rightRearPower  = drive + strafe - turn;

                telemetry.addData("Drive Mode", "MANUAL");
            }

            // Normalize motor power values so none exceed 1.0 magnitude
            double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                    Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower))));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftRearPower   /= max;
                rightRearPower  /= max;
            }

            // Apply power to drivetrain motors
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftRear.setPower(leftRearPower);
            rightRear.setPower(rightRearPower);

            // =========================================================
            // 2. Intake (Playstation A and B)
            // =========================================================

            boolean currentAState = gamepad1.a;
            boolean currentBState = gamepad1.b;

            // Toggle logic for Button A (Normal Power)
            if (currentAState && !previousAState) {
                isAOn = !isAOn; // Toggle A state
                if (isAOn) {
                    isBOn = false; // Automatically turn off B if A is turned on
                }
            }
            previousAState = currentAState;

            // Toggle logic for Button B (100% Power)
            if (currentBState && !previousBState) {
                isBOn = !isBOn; // Toggle B state
                if (isBOn) {
                    isAOn = false; // Automatically turn off A if B is turned on
                }
            }
            previousBState = currentBState;

            // Apply the power based on which toggle is active
            double intakePower = 0.0;
            if (isBOn) {
                intakePower = 1.0;  // 100% power for Button B
            } else if (isAOn) {
                intakePower = 0.65; // Normal power for Button A
            }

            intake.setPower(intakePower);

            // ------------------------------------------
            // Reverse Code
            // ------------------------------------------

            if (gamepad1.y) {
                intake.setDirection(DcMotor.Direction.FORWARD);
            } else{
                intake.setDirection(DcMotor.Direction.REVERSE);
            }


            if (gamepad2.y) {
                pollen.setDirection(DcMotor.Direction.FORWARD);
                nectar.setDirection(DcMotor.Direction.REVERSE);
            } else{
                pollen.setDirection(DcMotor.Direction.REVERSE);
                nectar.setDirection(DcMotor.Direction.FORWARD);
            }
            // ---------------------------------------------------------
            // 4. Gates
            // ---------------------------------------------------------

            // Nectar

            if (gamepad2.right_bumper){
                Nectar_gate.setPosition(0.75);
            } else{
                Nectar_gate.setPosition(1.0);
            }


            // Pollen

            if (gamepad2.left_bumper){
                Pollen_gate.setPosition(0.75);
            } else{
                Pollen_gate.setPosition(1.0);
            }

            // ---------------------------------------------------------
            // 5. Shooters
            // ---------------------------------------------------------

            // Nectar

            boolean current_right_trigger_state = gamepad2.right_trigger > 0.5;

            if (current_right_trigger_state && !previous_right_trigger_state) {
                isNectar_shooter_on = !isNectar_shooter_on;
            }
            previous_right_trigger_state = current_right_trigger_state;

            if (isNectar_shooter_on) {
                nectar.setVelocity(NECTAR_TARGET);
            } else {
                nectar.setVelocity(0);
            }

            boolean nectarAtSpeed = Math.abs(nectar.getVelocity() - NECTAR_TARGET) < VELOCITY_TOLERANCE;

            // Pollen

            boolean current_left_trigger_state = gamepad2.left_trigger > 0.5;

            if (current_left_trigger_state && !previous_left_trigger_state) {
                isPollen_shooter_on = !isPollen_shooter_on;
            }
            previous_left_trigger_state = current_left_trigger_state;

            if (isPollen_shooter_on) {
                pollen.setVelocity(POLLEN_TARGET);
            } else{
                pollen.setVelocity(0);
            }

            boolean pollenAtSpeed = Math.abs(pollen.getVelocity() - POLLEN_TARGET) < VELOCITY_TOLERANCE;

            // =========================================================
            // 6. TELEMETRY FEEDBACK
            // =========================================================
            telemetry.addData("Intake ON / OFF (X Button)", isMotorOn ? "ON" : "OFF");
            telemetry.addData("Intake Direction (Y Button)", intake.getDirection());
            telemetry.addData("Pollen Gate", isPollen_gate_up ? "UP" : "DOWN");
            telemetry.addData("Necttar Gate", isNectar_gate_up ? "UP" : "DOWN");
            telemetry.addData("Shooter Direction", nectar.getDirection());
            telemetry.addData("Nectar Velocity", nectar.getVelocity());
            telemetry.addData("Nectar Ready", nectarAtSpeed);
            telemetry.addData("Pollen Velocity", pollen.getVelocity());
            telemetry.addData("Pollen Ready", pollenAtSpeed);
            telemetry.update();
        }
    }
}