package org.firstinspires.ftc.teamcode.Library;

import android.util.Log;

//import com.qualcomm.hardware.adafruit.BNO055IMU;
//import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMUCalibration;

import java.util.Locale;

public abstract class MyOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public static BNO055IMU imu;
    public static DcMotor motorBL;
    public static DcMotor motorBR;
    public static DcMotor motorFL;
    public static DcMotor motorFR;
    public static DcMotor liftLeft;
    public static DcMotor liftRight;
    public static DcMotor manip;
    public static Servo jewelArm;
    public static Servo jewelHand;
    public static ColorSensor jewelColor;
    //public static DcMotor relic;
//    public static Servo relicGrabber;

    public static Orientation angles;
    public static Acceleration gravity;

    public static ModernRoboticsI2cRangeSensor rangeR;
    public static ModernRoboticsI2cRangeSensor rangeL;
    public static ModernRoboticsI2cRangeSensor rangeF;

    public double rangeLDis;
    public double rangeRDis;
    public double rangeFDis;

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            gravity  = imu.getGravity();
            }
        });
//
//        telemetry.addLine()
//                .addData("status", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getSystemStatus().toShortString();
//                    }
//                })
//                .addData("calib", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getCalibrationStatus().toString();
//                    }
//                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle); //Control Robot Pivot
                    }
                });

          telemetry.addLine()
                  .addData("tha Left 1 doe", new Func<String>() {
                      @Override public String  value() {
                          if (!Double.isNaN(rangeL.getDistance(DistanceUnit.INCH)) || rangeL.getDistance(DistanceUnit.INCH) < 1000) {
                              rangeLDis = rangeL.getDistance(DistanceUnit.INCH);
                              return Double.toString(rangeLDis);
                          }
                          return Double.toString(rangeLDis);
                      }
                  });
        telemetry.addLine()
                .addData("tha rite 1 doe", new Func<String>() {
                    @Override public String  value() {
                        if (!Double.isNaN(rangeR.getDistance(DistanceUnit.INCH)) || rangeR.getDistance(DistanceUnit.INCH) < 1000) {
                            rangeRDis = rangeR.getDistance(DistanceUnit.INCH);
                            return Double.toString(rangeRDis);
                        }
                        return Double.toString(rangeRDis);
                    }
                });

        telemetry.addLine()
                .addData("fdis", new Func<String>() {
                    @Override public String  value() {
                        if (!Double.isNaN(rangeF.getDistance(DistanceUnit.INCH)) || rangeF.getDistance(DistanceUnit.INCH) < 1000) {
                            rangeFDis = rangeF.getDistance(DistanceUnit.INCH);
                            return Double.toString(rangeFDis);
                        }
                        return Double.toString(rangeFDis);
                    }
                });

//          telemetry.addLine()
//                  .addData("RightD", new Func<String>() {
//                      @Override public String value() {
//                          return Double.toString(getRangeDistanceR()); //Control Robot Pivot
//                      }
//                  });

//                .addData("roll", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.secondAngle);
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("gravtiy", new Func<String>() {
//                    @Override public String value() {
//                        return gravity.toString();
//                    }
//                })
//                .addData("mag", new Func<String>() {
//                    @Override public String value() {
//                        return String.format(Locale.getDefault(), "%.3f",
//                                Math.sqrt(gravity.xAccel*gravity.xAccel
//                                        + gravity.yAccel*gravity.yAccel
//                                        + gravity.zAccel*gravity.zAccel));
//                    }
//                });
    }

    public void hMap(HardwareMap type) {
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

        rangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeR");
        rangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
        rangeF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeC");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
//        relic = hardwareMap.dcMotor.get("relic");
//        relicGrabber = hardwareMap.servo.get("relicGrabber");
    }

    public double getRangeDistanceL() {
        while (Double.isNaN(rangeL.getDistance(DistanceUnit.INCH)) || rangeL.getDistance(DistanceUnit.INCH) > 1000 || rangeL.getDistance(DistanceUnit.INCH) == 1.79769313486231570e+308) {
        }
        return rangeL.getDistance(DistanceUnit.INCH);
    }

    public double getRangeDistanceR() {
//        while ((rangeRDis > 1000 || Double.isNaN(rangeRDis)) && opModeIsActive()) {
//            rangeRDis = rangeR.getDistance(DistanceUnit.CM);
//        }
        return rangeRDis;
    }

    public void delay(long milliseconds) throws InterruptedException {
        if (milliseconds < 0)
            milliseconds = 0;
        Thread.sleep(milliseconds);
    }

    public void setMotors(double left, double right) {
        if (!opModeIsActive())
            return;

        motorFL.setPower(-left);
        motorBL.setPower(-left);
        motorFR.setPower(right);
        motorBR.setPower(right);
    }

    public void setMotorStrafe(double pow) { //Strafes right when positive.
        motorFL.setPower(-pow);
        motorBL.setPower(pow);
        motorFR.setPower(-pow);
        motorBR.setPower(pow);

    }

    public void stopMotors() {
        if (!opModeIsActive())
            return;

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }

    public void rangeMove(double pow, double inAway , double sensor)    { //Set pow negative to move backward.
        while ((sensor < inAway - .25) || (sensor > inAway + .25)) { //While sensor doesn't = tolerance, run.
            if (sensor > inAway) {
                setMotors(pow, pow);
            }
            if (sensor < inAway) {
                setMotors(-pow, -pow);
            }
        }
        stopMotors();
    }

    public void rangeMoveStrafe(double pow, double inAway , double sensor) { //Set pow to negative if we want to move left.
          while ((sensor < inAway - .25) || (sensor > inAway + .25)) {
            if (sensor > inAway) {
                setMotorStrafe(pow);
            }
            if (sensor < inAway) {
                setMotorStrafe(-pow);
            }
        }
        stopMotors();
    }

//    public void setMotorsMec() {
//        if (!opModeIsActive())
//            return;
//
//        if (gamepad1.left_trigger > .1) {
//            motorFL.setPower(-gamepad1.left_trigger);
//            motorBL.setPower(gamepad1.left_trigger);
//            motorFR.setPower(gamepad1.left_trigger);
//            motorBR.setPower(-gamepad1.left_trigger);
//        }
//        else if (gamepad1.right_trigger > .1){
//            motorFL.setPower(gamepad1.right_trigger);
//            motorBL.setPower(-gamepad1.right_trigger);
//            motorFR.setPower(-gamepad1.right_trigger);
//            motorBR.setPower(gamepad1.right_trigger);
//        }
//        else {
//            motorFL.setPower(0);
//            motorBL.setPower(0);
//            motorFR.setPower(0);
//            motorBR.setPower(0);
//        }
//    }

    // code for strafing using the dpad on driver 1 controller


    //Why do we have this method if we do the same thing in TeleOp ????
//    public void setMotorsMecDPAD(double left, double right, double lessenPower, double increasePower) {
//        if (!opModeIsActive())
//            return;
//        double less = lessenPower;
//        double increase = increasePower;
//
//        if (gamepad1.dpad_up &&  left <= .75 && right <= .75) {
//            left += increasePower;
//            right += increasePower;
//        }
//
//        else if (gamepad1.dpad_down && left >= 0.25 && right >= 0.25){
//            left += lessenPower;
//            right += lessenPower;
//
//        }
//
//        if (gamepad1.dpad_left) {
//            motorFL.setPower(left);
//            motorBL.setPower(-left);
//            motorFR.setPower(-right);
//            motorBR.setPower(right);
//        }
//        else if (gamepad1.dpad_right){
//            motorFL.setPower(-left);
//            motorBL.setPower(left);
//            motorFR.setPower(right);
//            motorBR.setPower(-right);
//        }
//        else {
//            motorFL.setPower(0);
//            motorBL.setPower(0);
//            motorFR.setPower(0);
//            motorBR.setPower(0);
//        }
//
//    }

    public void slowDown(double reduction) {
        //first reduction making power 0.5
        while (motorFL.getPower() > 0.05 && motorBL.getPower() > 0.05 && motorFR.getPower() > 0.05 && motorFL.getPower() > 0.05) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            motorFL.setPower(motorFL.getPower() * reduction);
            motorBL.setPower(motorBL.getPower() * reduction);
            motorFR.setPower(motorFL.getPower() * reduction);
            motorBR.setPower(motorBL.getPower() * reduction);
        }

        if (motorFL.getPower() < 0.05 && motorBL.getPower() < 0.05 && motorFR.getPower() < 0.05 && motorFL.getPower() < 0.05) {
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);
        }

    }

//    public void mecAutoLeft(double left, double right, double distance ,int tim ) {
//        if (!opModeIsActive())
//            return;
//        ElapsedTime time = new ElapsedTime();
//
//        time.reset();
//        resetStartTime();
//
//
//        if (distance > 0 && getRangeDistanceRight() < distance && time.milliseconds() < tim) {
//            motorFL.setPower(-left);
//            motorBL.setPower(left);
//            motorFR.setPower(right);
//            motorBR.setPower(-right);
//        }
//        else {
//            motorFL.setPower(0);
//            motorBL.setPower(0);
//            motorFR.setPower(0);
//            motorBR.setPower(0);
//        }
//    }

//    public void mecAutoRight(double left, double right, double distance ,int tim) {
//        if (!opModeIsActive())
//            return;
//
//        ElapsedTime time = new ElapsedTime();
//
//        time.reset();
//        resetStartTime();
//
//        if (distance > 0 && getRangeDistanceRight() < distance && time.milliseconds() < tim) {
//            motorFL.setPower(left);
//            motorBL.setPower(-left);
//            motorFR.setPower(-right);
//            motorBR.setPower(right);
//        }
//        else {
//            motorFL.setPower(0);
//            motorBL.setPower(0);
//            motorFR.setPower(0);
//            motorBR.setPower(0);
//        }
//    }

//    public double getRangeDistanceRight() {
//        double value = rangeR.getDistance(DistanceUnit.CM);
//        if (value != 255)
//            rangeDistance = value;
//        return rangeDistance;
//    }

    //    public void depositBlockAuto(double dep) {
//        if (dep > 0.1) {
//            manip.setPower(dep);
//        } else if (dep < -0.1) {
//            manip.setPower(-dep);
//        } else {
//            manip.setPower(0);
//        }
//        manip.setPower(0);
//
//    }
//
    public void jewelKnockerRed() {
        jewelArm.setPosition(.6);
        jewelHand.setPosition(.45);
        sleep(2000);
        jewelArm.setPosition(.15);
        sleep(2000);

        if (jewelColor.red() > jewelColor.blue()) {
            jewelHand.setPosition((.3));

        } else if (jewelColor.red() < jewelColor.blue()) {
            jewelHand.setPosition((.6));
        }

        sleep(1000);
        jewelArm.setPosition(.6);
        jewelHand.setPosition(.45);
        sleep(3000);

        jewelHand.setPosition(.3);
        sleep(1000);
    }
    // int colorDif = jewelColor.red() - jewelColor.blue();

    public void jewelKnockerBlue() {
        jewelArm.setPosition(.6);
        jewelHand.setPosition(.45);
        sleep(2000);
        jewelArm.setPosition(.15);
        sleep(2000);

        if (jewelColor.red() < jewelColor.blue()) {
            jewelHand.setPosition((.3));

        } else if (jewelColor.red() > jewelColor.blue()) {
            jewelHand.setPosition((.6));
        }

        sleep(1000);
        jewelArm.setPosition(.6);
        jewelHand.setPosition(.45);
        sleep(1000);

        jewelHand.setPosition(.3);
        sleep(1000);
    }

//    public double getUltraDistance() {
//        double value = ultra.cmUltrasonic();
//        if (value != 255)
//            ultraDistance = value;
//        return ultraDistance;
//    }
//
//    public void setManip(double pow) {
//        if (!opModeIsActive())
//            return;
//
//        manip.setPower(pow);
//    }

    public void setServoSlow(Servo servo, double pos) throws InterruptedException {
        double currentPosition = servo.getPosition();

        if (currentPosition - pos > 0) {
            for (; currentPosition > pos; currentPosition -= .005) {
                servo.setPosition(currentPosition);
                delay(1);
                idle();
            }
        } else for (; currentPosition < pos; currentPosition += .005) {
            servo.setPosition(currentPosition);
            delay(1);
            idle();
        }
    }

//    public void turnCorr(double pow, double deg) throws InterruptedException {
//        turnCorr(pow, deg, 8000);
//    }

    public double getHead() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    public void turnCorr(double pow, double deg, int tim) throws InterruptedException {
        if (!opModeIsActive())
            return;

        double newPow;

        ElapsedTime time = new ElapsedTime();
        getHead();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)); // startP is the current position

        delay(100);
        time.reset();

        if (deg > 0) {
            while ((deg > currPos && time.milliseconds() < tim)) {
                newPow = pow * (Math.abs(deg - currPos) / 80);

                if (newPow < .15)
                    newPow = .15;

                setMotors(-newPow, newPow);
                currPos = getHead();
                telemetry.addData("Gyro", currPos);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        } else {
            while (deg < currPos && time.milliseconds() < tim) {
                newPow = pow * (Math.abs(deg - currPos) / 80);

                if (newPow < .15)
                    newPow = .15;
                setMotors(newPow, -newPow);
                currPos = getHead();
                telemetry.addData("Gyro", currPos);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        }

        stopMotors();

        if (currPos > deg) {
            while (opModeIsActive() && deg < currPos) {
                newPow = pow * (Math.abs(deg - currPos) / 80);

                if (newPow < .15)
                    newPow = .15;
                setMotors(.15, -.15);
                currPos = getHead();
                telemetry.addData("Gyro", currPos);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && deg > currPos) {
                newPow = pow * (Math.abs(deg - currPos) / 80);

                if (newPow < .15)
                    newPow = .15;

                setMotors(-.15, .15);
                currPos = getHead();
                telemetry.addData("Gyro", currPos);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        }
        stopMotors();
    }
}