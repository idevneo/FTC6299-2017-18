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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


@Autonomous(name="RedRightDrive", group="Linear Opmode")

public class AutoRedDrive extends MyOpMode {

    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;

    DcMotor liftLeft;
    DcMotor liftRight;

    DcMotor manip;

    ColorSensor jewelColor;

    Servo jewelArm;
    Servo jewelHand;

//    ModernRoboticsI2cRangeSensor rangeR;
//    ModernRoboticsI2cRangeSensor rangeL;

    Orientation angles;
    protected String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

        liftLeft = hardwareMap.dcMotor.get("liftL");
        liftRight = hardwareMap.dcMotor.get("liftR");
        manip = hardwareMap.dcMotor.get("manip");

        jewelColor = hardwareMap.get(ColorSensor.class, "jewelColor");

        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelHand = hardwareMap.servo.get("jewelHand");

//        rangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeR");
//        rangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeL");

         //heading - -numerical value or it = null


        waitForStart();
        runtime.reset();


        turnCorr(.2, 90);
//        while(opModeIsActive()) {
//            telemetry.addData("Red  ", jewelColor.red());
//            telemetry.addData("Green", jewelColor.green());
//            telemetry.addData("Blue ", jewelColor.blue());
//            telemetry.update();
//        }

        // run until the end of the match (driver presses STOP)
//            jewelArm.setPosition(.6);
//            jewelHand.setPosition(.45);
//            sleep(2000);
//            jewelArm.setPosition(.15);
//            sleep(2000);
//
//            if (jewelColor.red() > jewelColor.blue()) {
//                jewelHand.setPosition((.3));
//
//            } else if (jewelColor.red() < jewelColor.blue()) {
//                jewelHand.setPosition((.6));
//            }
//
//            sleep(1000);
//            jewelArm.setPosition(.6);
//            jewelHand.setPosition(.45);
//            sleep(3000);
//
//            jewelHand.setPosition(.3);
//            sleep(1000);
//
//
//             manip.setPower(1);
//             Thread.sleep(300);
//             manip.setPower(0);
//
//            motorFL.setPower(-0.5);
//            motorBL.setPower(-0.5);
//            motorFR.setPower(0.5);
//            motorBR.setPower(0.5);
//            Thread.sleep(1050);
//
//            motorFL.setPower(0);
//            motorBL.setPower(0);
//            motorFR.setPower(0);
//            motorBR.setPower(0);
//
//            sleep(700);

//        while (rangeR.getDistance(DistanceUnit.CM) < 91 && runtime.milliseconds() < 6000) {
//            motorFL.setPower(-.75);
//            motorBL.setPower(.75);
//            motorFR.setPower(.75);
//        }
//            motorFL.setPower(0);
//            motorBL.setPower(0);
//            motorFR.setPower(0);
//            motorBR.setPower(0);
//            sleep(5000);
//
//        //strafing left
//        motorFL.setPower(.75);
//        motorBL.setPower(-.75);
//        motorFR.setPower(.75);
//        motorBR.setPower(-.75);
//
//        Thread.sleep(400);
//
//        motorFL.setPower(0);
//        motorBL.setPower(0);
//        motorFR.setPower(0);
//        motorBR.setPower(0);
//
//        sleep(500);
//
//        manip.setPower(-1);
//        Thread.sleep(700);
//        manip.setPower(0);
//
//        sleep(1000);
//
//        motorFL.setPower(0.5);
//        motorBL.setPower(0.5);
//        motorFR.setPower(-0.5);
//        motorBR.setPower(-0.5);
//
//        Thread.sleep(150);
//
//        motorFL.setPower(0);
//        motorBL.setPower(0);
//        motorFR.setPower(0);
//        motorBR.setPower(0);

        }

}

