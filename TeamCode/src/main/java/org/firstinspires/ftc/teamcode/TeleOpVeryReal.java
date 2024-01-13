package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    }

    @Override
    public void loop() {

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

        boolean slide_up = gamepad2.dpad_up;
        boolean slide_down = gamepad2.dpad_down;
        if (slide_up == true) {
            slide_right.setPower(1);
        }
        else if (slide_down == true) {
            slide_right.setPower(-1);
        }
        else {
            slide_right.setPower(0);
        }
    }
}
