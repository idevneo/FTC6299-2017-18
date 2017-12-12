package org.firstinspires.ftc.teamcode.Library;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


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

    public VuforiaLocalizer vuforia;
    public char column;

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
//        telemetry.addLine()
//                .addData("VumarkGoal", new Func<String>() {
//                    @Override public String  value() {
//                        return Character.toString(getVuMark());
//                    }
//                });
          telemetry.addLine()
                  .addData("Left", new Func<String>() {
                      @Override public String  value() {
                          double localRange;
                          double sensor = 0;
                          localRange = rangeL.getDistance(DistanceUnit.INCH);
                          if (!Double.isNaN(localRange) && (localRange < 1000)) {
                              sensor = localRange;
                          }
                          return Double.toString(sensor);
                      }
                  });
        telemetry.addLine()
                .addData("Right", new Func<String>() {
                    @Override public String  value() {
                        double localRange;
                        double sensor = 0;
                        localRange = rangeR.getDistance(DistanceUnit.INCH);
                        if (!Double.isNaN(localRange) && (localRange < 1000)) {
                            sensor = localRange;
                        }
                        return Double.toString(sensor);
                    }
                });

        telemetry.addLine()
                .addData("Front", new Func<String>() {
                    @Override public String  value() {
                        double localRange;
                        double sensor = 0;
                        localRange = rangeF.getDistance(DistanceUnit.INCH);
                        if (!Double.isNaN(localRange) && (localRange < 1000)) {
                            sensor = localRange;
                        }
                        return Double.toString(sensor);
                    }
                });

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

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void manipAuto(double pow) {
        if (!opModeIsActive())
            return;
        if (pow > 0) {
            manip.setPower(pow);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            manip.setPower(0);
        } else if (pow < 0) {
            manip.setPower(pow);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            manip.setPower(0);
        } else {
            manip.setPower(0);
        }

    }
    public char getVuMark() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXb/g5n/////AAAAGSUed2rh5Us1jESA1cUn5r5KDUqTfwO2woh7MxjiLKSUyDslqBAgwCi0Qmc6lVczErnF5TIw7vG5R4TJ2igvrDVp+dP+3i2o7UUCRRj/PtyVgb4ZfNrDzHE80/6TUHifpKu4QCM04eRWYZocWNWhuRfytVeWy6NSTWefM9xadqG8FFrFk3XnvqDvk/6ZAgerNBdq5SsJ90eDdoAhgYEee40WxasoUUM9YVMvkWOqZgHSuraV2IyIUjkW/u0O+EkFtTNRUWP+aZwn1qO1H4Lk07AJYe21eqioBLMdzY7A8YqR1TeQ//0WJg8SFdXjuGbF6uHykBe2FF5UeyaehA0iTqfPS+59FLm8y1TuUt57eImq";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //telemetry.addData(">", "Press Play to start");

        relicTrackables.activate();


        // copy pasta from the ftc ppl
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);


        telemetry.addData("VuMark ", vuMark);
        telemetry.update();
        if (vuMark == RelicRecoveryVuMark.CENTER)
            column = 'C';
        else if (vuMark == RelicRecoveryVuMark.LEFT)
            column = 'L';
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
            column = 'R';
        else if (vuMark == RelicRecoveryVuMark.UNKNOWN)
            column = 'U';
        return column;

    }

    public void vfMovePerp(char rb, ModernRoboticsI2cRangeSensor sensorVar) {
        double centerDis = 30;
        double kC = 0;
        if (rb == 'r')
            kC = 8.5;
        if (rb == 'b')
            kC = -8.5;
        if (column == 'L') {
            rangeMoveStrafe((centerDis + kC) , sensorVar);
        } else if (column == 'R') {
            rangeMoveStrafe((centerDis - kC), sensorVar);
        } else if (column == 'C') {
            rangeMoveStrafe(centerDis, sensorVar);
        } else if (column == 'U'){
            rangeMoveStrafe(centerDis, sensorVar);
        }

    }



//    public void rangeMove(double pow, double inAway, ModernRoboticsI2cRangeSensor sensorVar)    { //Set pow negative to move backward.
//        double sensor = sensorVar.getDistance(DistanceUnit.INCH);
//
//        while ((sensor < inAway - .25) || (sensor > inAway + .25)) { //While sensor doesn't = tolerance, run.
//            double localRange;
//            localRange = sensorVar.getDistance(DistanceUnit.INCH);
//            if (!Double.isNaN(localRange) && (localRange < 1000)) {
//                sensor = localRange;
//            }
//
//            telemetry.addData("SensorValue", sensor); //optional telemetry
//            telemetry.update();
//
//            if (sensor > inAway) {
//                setMotors(pow, pow);
//            }
//            if (sensor < inAway) {
//                setMotors(-pow, -pow);
//            }
//        }
//        stopMotors();
//    }

    public void rangeMovePID(double inAway, ModernRoboticsI2cRangeSensor sensorVar)    {
        double sensor = sensorVar.getDistance(DistanceUnit.INCH);
//        double differenceDis;
//        double kP = .025; //to be determined
        double pow;
        double localRange;
        while (((sensor < inAway - .25) || (sensor > inAway + .25)) && opModeIsActive()) { //While sensor doesn't = tolerance, run.
            localRange = sensorVar.getDistance(DistanceUnit.INCH);
            while (Double.isNaN(localRange) || (localRange > 1000)) {
                localRange = sensorVar.getDistance(DistanceUnit.INCH);
            }
            sensor = localRange;

//            differenceDis = Math.abs(sensor - inAway);
//            pow = differenceDis*kP;
            pow = .125;

//            if (pow > .2)
//                pow = .2;
//            if (pow < .1)
//                pow = .1;
            telemetry.addData("SensorValue", sensor); //optional telemetry
            telemetry.update();

            if (sensor > inAway) {
                setMotors(pow, pow);
            }
            if (sensor < inAway) {
                setMotors(-pow, -pow);
            }
        }
        stopMotors();
    }





    public void rangeMoveStrafe(double inAway , ModernRoboticsI2cRangeSensor sensorVar) { //Set pow to negative if we want to move left.
        double sensor = sensorVar.getDistance(DistanceUnit.INCH);
//        double differenceDis;
//        double kP = .025; //to be determined
        double pow;
        double localRange;
          while (((sensor < inAway - .5) || (sensor > inAway + .5))&& opModeIsActive()) {
              localRange = sensorVar.getDistance(DistanceUnit.INCH);
              while (Double.isNaN(localRange) || (localRange > 1000)) {
                  localRange = sensorVar.getDistance(DistanceUnit.INCH);
              }
              sensor = localRange;
              //differenceDis = Math.abs(sensor - inAway);
              //pow = differenceDis*kP;
              pow = 0.15;
//              if (pow > .2) edit once working on single power
//                  pow = .2;
//              if (pow < .19)
//                  pow = .19;

            if (sensor > inAway) {
                setMotorStrafe(pow);
            }
            if (sensor < inAway) {
                setMotorStrafe(-pow);

            }
              telemetry.addData("rightwhile", sensor);
              telemetry.update();
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

    public double getDiff(double angle) {
        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        return   currPos - angle;
    }
    public Double gyroVal() {
        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        return currPos;
    }
    public void turnPID(double angle) throws InterruptedException {
        double kP = .20/90;
        double min = -.2;
        double max = .2;
        double changeCon = .04;
        double PIDchange;
        double angleDiff = getDiff(angle);
        double oldDiff = angleDiff;
        int counter = 0;
        double startAngle = gyroVal();
        while ((Math.abs(angleDiff) <= Math.abs(oldDiff)) && opModeIsActive()) {
            PIDchange = angleDiff * kP;

            if (PIDchange < 0) {
                motorFR.setPower(Range.clip(-PIDchange + changeCon, min, max));
                motorBR.setPower(Range.clip(-PIDchange + changeCon, min, max));
                motorFL.setPower(Range.clip(-PIDchange + changeCon, min, max));
                motorBL.setPower(Range.clip(-PIDchange + changeCon, min, max));
            }
            else {
                motorFR.setPower(Range.clip(PIDchange - changeCon, min, max));
                motorBR.setPower(Range.clip(PIDchange - changeCon, min, max));
                motorFL.setPower(Range.clip(PIDchange - changeCon, min, max));
                motorBL.setPower(Range.clip(PIDchange - changeCon, min, max));
            }

            sleep(250);
            telemetry.addData("gyroStart", startAngle);
            telemetry.addData("counter", counter++);
            telemetry.addData("GyroVal", gyroVal());
            telemetry.addData("GyroDiff",getDiff(angle));
            telemetry.addData("Pow", -PIDchange - changeCon);
            telemetry.update();

            oldDiff = angleDiff;
            angleDiff = getDiff(angle);
        }
        stopMotors();
    }

    public void turn(double pow, double deg) throws InterruptedException {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        if (deg > 0) {
            while ((currPos > (deg + .25)) || (currPos < (deg-.25))){
                if (currPos > (deg + .25))
                    setMotors(-pow, pow);
                if (currPos < (deg -.25))
                    setMotors(pow, -pow);

                currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                idle();
            }
            stopMotors();

        }
        if (deg < 0) {
            while ((currPos > (deg + .25)) || (currPos < (deg-.25))){
                if (currPos > (deg + .25))
                    setMotors(-pow, pow);
                if (currPos < (deg -.25))
                    setMotors(pow, -pow);
                currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                idle();
            }
            stopMotors();
        }
    }
    public void turnCorr(double pow, double deg, int tim) throws InterruptedException {
        if (!opModeIsActive())
            return;

        double newPow;

        ElapsedTime time = new ElapsedTime();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)); // startP is the current position

        delay(100);
        time.reset();

        if (deg > 0) {
            while ((deg > currPos && time.milliseconds() < tim)) {
                newPow = pow * (Math.abs(deg - currPos) / 80);

                if (newPow < .15)
                    newPow = .1;

                setMotors(-newPow, newPow);
                currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Gyro", currPos);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        } else {
            while (deg < currPos && time.milliseconds() < tim) {
                newPow = pow * (Math.abs(deg - currPos) / 80);

                if (newPow < .15)
                    newPow = .1;
                setMotors(newPow, -newPow);
                currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
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
                    newPow = .1;
                setMotors(newPow, -newPow);
                currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Gyro", currPos);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && deg > currPos) {
                newPow = pow * (Math.abs(deg - currPos) / 80);

                if (newPow < .15)
                    newPow = .1;

                setMotors(-newPow, newPow);
                currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Gyro", currPos);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        }
        stopMotors();
    }
}