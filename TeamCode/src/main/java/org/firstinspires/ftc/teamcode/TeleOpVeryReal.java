package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOpVeryReal extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor slide_right = null;
    private DcMotor slide_left  = null;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left   = hardwareMap.get(DcMotor.class, "FrontLeft");
        front_right  = hardwareMap.get(DcMotor.class, "FrontRight");
        back_left	 = hardwareMap.get(DcMotor.class, "BackLeft");
        back_right   = hardwareMap.get(DcMotor.class, "BackRight");
        slide_right  = hardwareMap.get(DcMotor.class, "SlideRight");
        slide_left  = hardwareMap.get(DcMotor.class, "SlideLeft");

        slide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_left.setDirection(DcMotorSimple.Direction.REVERSE);

        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

        // DRIVE CODE
        double y = gamepad1.right_stick_x; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.left_stick_y * -1;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max((Math.abs(y) + Math.abs(x) + Math.abs(rx)), 1);
        double frontLeftPower = ((y + x + rx) / denominator);
        double backLeftPower = ((y - x + rx) / denominator);
        double frontRightPower = ((y - x - rx) / denominator);
        double backRightPower = ((y + x - rx) / denominator);


        front_left.setPower(frontLeftPower);
        back_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        back_right.setPower(backRightPower);

        /*
         LINEAR SLIDE CODE
        */
        boolean slide_high_pos = gamepad2.dpad_up;
        boolean slide_low_pos = gamepad2.dpad_down;
        if (slide_high_pos) {
            slide_right.setTargetPosition(10);
            slide_left.setTargetPosition(10);
        }
        else if (slide_low_pos) {
            slide_right.setTargetPosition(0);
            slide_left.setTargetPosition(0);
        }
        telemetry.addData("SlidePosition", slide_right.getCurrentPosition());
    }
}
