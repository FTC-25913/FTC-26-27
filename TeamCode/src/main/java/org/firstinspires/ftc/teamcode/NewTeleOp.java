package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="New BioBuzz TeleOp", group="Linear Opmode")
public class NewTeleOp extends LinearOpMode {

    private DcMotor intake = null;
    private boolean previousAState = false;
    private boolean isIntakeOn = false;

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            boolean currentAState = gamepad1.a;

            if (currentAState && !previousAState) {
                isIntakeOn = !isIntakeOn;
            }

            previousAState = currentAState;

            if (isIntakeOn) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0);
            }
        }
    }
}
