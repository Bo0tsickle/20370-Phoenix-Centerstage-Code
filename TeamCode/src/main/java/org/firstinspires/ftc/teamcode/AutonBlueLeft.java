/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous

public class AutonBlueLeft extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_right = null;
    private DcMotor front_left = null;
    private DcMotor back_right = null;
    private DcMotor back_left = null;

    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 96 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    void drive(int front_right_target_IN, int front_left_target_IN, int back_right_target_IN, int back_left_target_IN) {
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int front_right_target = (int)(front_right_target_IN * DRIVE_COUNTS_PER_IN);
        int front_left_target = (int)(front_left_target_IN * DRIVE_COUNTS_PER_IN);
        int back_right_target = (int)(back_right_target_IN * DRIVE_COUNTS_PER_IN);
        int back_left_target = (int)(back_left_target_IN * DRIVE_COUNTS_PER_IN);

        front_right.setTargetPosition(front_right_target);
        front_left.setTargetPosition(front_left_target);
        back_right.setTargetPosition(back_right_target);
        back_left.setTargetPosition(back_left_target);

        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(0.85);
        front_left.setPower(0.85);
        back_right.setPower(0.85);
        back_left.setPower(0.85);

        while (front_right.isBusy() || front_left.isBusy() || back_right.isBusy() || back_left.isBusy()) {
            telemetry.addLine("Moving!");
            telemetry.update();
        }

        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        front_right = hardwareMap.get(DcMotor.class, "FrontRight");
        front_left = hardwareMap.get(DcMotor.class, "FrontLeft");
        back_right = hardwareMap.get(DcMotor.class, "BackRight");
        back_left = hardwareMap.get(DcMotor.class, "BackLeft");
        DistanceSensor sensor_left = hardwareMap.get(DistanceSensor.class, "DistanceSensorLeft");
        DistanceSensor sensor_right = hardwareMap.get(DistanceSensor.class, "DistanceSensorRight");

        CRServo grip_spin = hardwareMap.get(CRServo.class, "GripSpin");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        boolean left_detected = false;
        boolean middle_detected = false;
        boolean right_detected = false;
        if (sensor_left.getDistance(DistanceUnit.INCH) < 48) {
            left_detected = true;
        }
        else if (sensor_right.getDistance(DistanceUnit.INCH) < 48) {
            middle_detected = true;
        }
        else {
            right_detected = true;
        }

        if (left_detected) {
            drive(27, 27, 27, 27); // forward
            drive (22, -22, 22, -22); // turn left
            grip_spin.setPower(1.0);
        }
        else if (middle_detected) { // works
            drive(41, 41, 41, 41); // forward
            grip_spin.setPower(1.0);
        }
        else if (right_detected) {
            drive(27, 27, 27, 27); // forward
            drive (-22, 22, -22, 22); // turn right
            grip_spin.setPower(1.0);
        }
    }
}

