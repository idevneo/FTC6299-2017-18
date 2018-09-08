package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

public abstract class MyOpModeNEW extends LinearOpMode {

    public static BNO055IMU imu;
    public VuforiaLocalizer vuforia;

    public static DcMotor motorBL;
    public static DcMotor motorBR;
    public static DcMotor motorFL;
    public static DcMotor motorFR;
    public static DcMotor motorML;
    public static DcMotor motorMR;




    public static DcMotor liftLeft;
    public static DcMotor liftRight;
    public static DcMotor manip;
    public static Servo manipWall;
    public static DcMotor relicDrive;
    public static Servo jewelArm;
    public static Servo jewelHand;
    public static Servo relicFlip;
    public static Servo relicHand;
    public static ColorSensor jewelColor;

    public static Orientation angles;
    public static Acceleration gravity;

    public static ModernRoboticsI2cRangeSensor rangeR;
    public static ModernRoboticsI2cRangeSensor rangeL;
    public static ModernRoboticsI2cRangeSensor rangeF;

    public char column;
    public boolean align;
    public double lastPow;

    public double gyroError = 0;
    public ElapsedTime xDelay = new ElapsedTime();

    public String formatAngle(AngleUnit angleUnit, double angle) { //Formats the IMU Angle data into strings that can pass into telemetry.
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees) { //Formats the IMU Degree data into strings that can pass into telemetry.
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void composeTelemetry() { //Method that holds a majority of sensor telemetry for testing.
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
        telemetry.addLine()
                .addData("Angle", new Func<String>() { //Position of the robots angle.
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle); //Control Robot Pivot
                    }
                });
        telemetry.addLine()
                .addData("Left", new Func<String>() { //Left Range Sensor Telemetry.
                    @Override
                    public String value() {
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
                .addData("Right", new Func<String>() { //Right Range Sensor Telemetry.
                    @Override
                    public String value() {
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
                .addData("Front", new Func<String>() { //Front Range Sensor Telemetry.
                    @Override
                    public String value() {
                            double localRange;
                            double sensor = 0;
                            localRange = rangeF.getDistance(DistanceUnit.INCH);
                            if (!Double.isNaN(localRange) && (localRange < 1000)) {
                                sensor = localRange;
                            }
                            return Double.toString(sensor);
                        }
                });
    }

    public void hMap(HardwareMap type) { //Initialization of the Robot's hardware map in autonomous.
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

        liftLeft = hardwareMap.dcMotor.get("liftL");
        liftRight = hardwareMap.dcMotor.get("liftR");
        manip = hardwareMap.dcMotor.get("manip");
        manipWall = hardwareMap.servo.get("manipWall");
        jewelColor = hardwareMap.get(ColorSensor.class, "jewelColor");
        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelHand = hardwareMap.servo.get("jewelHand");
        relicDrive = hardwareMap.dcMotor.get("relicDrive");
        relicFlip = hardwareMap.servo.get("relicFlip");
        relicHand = hardwareMap.servo.get("relicHand");

        rangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeR");
        rangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
        rangeF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeC");

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        gyroVuforiaInit();

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void gyroVuforiaInit () {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AXb/g5n/////AAAAGSUed2rh5Us1jESA1cUn5r5KDUqTfwO2woh7MxjiLKSUyDslqBAgwCi0Qmc6lVczErnF5TIw7vG5R4TJ2igvrDVp+dP+3i2o7UUCRRj/PtyVgb4ZfNrDzHE80/6TUHifpKu4QCM04eRWYZocWNWhuRfytVeWy6NSTWefM9xadqG8FFrFk3XnvqDvk/6ZAgerNBdq5SsJ90eDdoAhgYEee40WxasoUUM9YVMvkWOqZgHSuraV2IyIUjkW/u0O+EkFtTNRUWP+aZwn1qO1H4Lk07AJYe21eqioBLMdzY7A8YqR1TeQ//0WJg8SFdXjuGbF6uHykBe2FF5UeyaehA0iTqfPS+59FLm8y1TuUt57eImq";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
        Gparameters.mode = BNO055IMU.SensorMode.IMU;
        Gparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Gparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Gparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        Gparameters.loggingEnabled = true;
        Gparameters.loggingTag = "IMU";
        Gparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(Gparameters);
    }

    public void hMapT(HardwareMap type) { //Initialization of the Robot's hardware map in TeleOp.
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

        liftLeft = hardwareMap.dcMotor.get("liftL");
        liftRight = hardwareMap.dcMotor.get("liftR");
        manip = hardwareMap.dcMotor.get("manip");
        manipWall = hardwareMap.servo.get("manipWall");
        jewelColor = hardwareMap.get(ColorSensor.class, "jewelColor");
        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelHand = hardwareMap.servo.get("jewelHand");
        relicDrive = hardwareMap.dcMotor.get("relicDrive");
        relicFlip = hardwareMap.servo.get("relicFlip");
        relicHand = hardwareMap.servo.get("relicHand");

        rangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeR");
        rangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
        rangeF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeC");

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void delay(long milliseconds) throws InterruptedException { //Delays the code via a sleep time.
        if (milliseconds < 0)
            milliseconds = 0;
        Thread.sleep(milliseconds);
    }

    public void setMotorsAll(double linear, double strafe, double turn) {

        motorFL.setPower(-linear - turn - strafe);
        motorBL.setPower(-linear - turn + strafe);
        motorFR.setPower(linear - turn - strafe);
        motorBR.setPower(linear - turn + strafe);
    }

    public void setMotors(double left, double right) { //Moves forward when both values are positive.
        if (!opModeIsActive())
            return;

        motorFL.setPower(-left);
        motorBL.setPower(-left);
        motorFR.setPower(right);
        motorBR.setPower(right);
    }

    public void stopMotors() { //Stops the motors.
        if (!opModeIsActive())
            return;

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }

    public void vfValue() { //Finds the column we need to place the glyph in using Vuforia.
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        column = 'U';

        ElapsedTime time = new ElapsedTime();
        time.reset();
        resetStartTime();

        while ((time.milliseconds() < 2000) && opModeIsActive() && (column == 'U')) { //Finding the Cryptokey
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            switch (vuMark) { //Assigns the Cryptokey Image to a value we can use later.
                case UNKNOWN:
                    column = 'U';
                    break;
                case CENTER:
                    column= 'C';
                    break;
                case LEFT:
                    column= 'L';
                    break;
                case RIGHT:
                    column = 'R';
                    break;
            }
        }
        telemetry.addData("Column", column);
        telemetry.update();
    }

//    public void vfMovePerp(char rb, ModernRoboticsI2cRangeSensor sensorVar, double bSwitch) { //For Perpendicular Cryptobox movement;
//        // strafes to the correct column using range sensors via our vfValue.
//        double centerDis = 26.75; //needs to be tested
//        double kC = 0;
//        switch (rb) { //Determines whether our calculations need to calculate for the red side or blue side.
//            case 'r':
//                kC = 6; //need to be adjusted
//                break;
//            case 'b':
//                kC = -6; //need to be adjusted.
//                break;
//        }
//        switch (column) { //Moves accordingly to the variable column.
//            case 'L':
//                rangeMoveStrafe((centerDis + kC), sensorVar, bSwitch);
//                break;
//            case 'R':
//                rangeMoveStrafe((centerDis - kC), sensorVar, bSwitch);
//                break;
//            case 'C':case 'U':
//                break;
//        }
//    }

    public void rangeMovePID(double inAway, ModernRoboticsI2cRangeSensor sensorVar) { //Moving forward/backwards using a Range Sensor.
        double sensor = sensorVar.getDistance(DistanceUnit.INCH);
        double pow;
        double localRange;
        while (((sensor < inAway - .35) || (sensor > inAway + .35)) && opModeIsActive() && align) { //While sensor isn't in the desired position, run.
            localRange = sensorVar.getDistance(DistanceUnit.INCH);
            if (gamepad1.x && xDelay.time() > .5) {
                stopMotors();
                align = false;
                xDelay.reset();
            }
            while ((Double.isNaN(localRange) || (localRange > 1000)) && opModeIsActive() && align) {
                if (gamepad1.x && xDelay.time() > .5) {
                    align = false;
                    xDelay.reset();
                 }
                 else if (!gamepad1.atRest()) { //Supposed to make the loop exit if joysticks are pressed.
                    stopMotors();
                    align = false;

                }

                //If a faulty value is detected, don't update our used variable till a good one is found.
                localRange = sensorVar.getDistance(DistanceUnit.INCH);
            }
            sensor = localRange; //Sets all working an  d usable values into a variable we can utilize.


            pow = .15;

            if (sensor > inAway) { //If the sensor value is greater than the target, move forward.
                setMotors(pow, pow);
            }
            if (sensor < inAway) { //If the sensor value is lower than than the target, move backwards.
                setMotors(-pow, -pow);
            }

            telemetry.addData("SensorValue", sensor); //Optional Telemetry
            telemetry.update();
        }
        stopMotors();
    }

    public void turnCorr(double pow, double deg, int timer) throws InterruptedException { //Turns to a desired angle using the IMU.
        if (!opModeIsActive()) //if the OpMode is not active, don't run.
            return;

        double newPow;
        double error;
        double errorMove;

        ElapsedTime time = new ElapsedTime();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)); // currPos is the current position

        delay(100);
        time.reset();

        while ((currPos > deg + 1 || currPos < deg - 1) && (time.milliseconds() < timer) && opModeIsActive()) { //While sensor isn't in the desired angle position, run.
            error = deg - currPos; //Finding how far away we are from the target position.
            errorMove = Math.abs(deg - currPos);
            if (error > 180) {
                error = error - 360;
            } else if (error < -180) {
                error = error + 360;
            }

            newPow = pow * (Math.abs(error) / 70); //Using the error to calculate our power.
            if (newPow < .15)
                newPow = .1;

            //The following code allows us to turn in the direction we need, and if we cross the axis
            //at which 180 degrees becomes -180, our robot can turn back in the direction which is closest
            //to the position we wish to be at (We won't make a full rotation to get to -175, if we hit 180).
            if (currPos < deg) {
                if (errorMove < 180) {
                    setMotors(-newPow, newPow); //Turns left
                }
                if (errorMove > 180) {
                    setMotors(newPow, -newPow); //Turns right if we go past the pos/neg mark.
                }
            } else if (currPos > deg) {
                if (errorMove < 180) {
                    setMotors(newPow, -newPow); //Turns right
                }
                if (errorMove > 180) {
                    setMotors(-newPow, newPow); //Turns left if we go past the pos/neg mark.
                }
            }
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("Gyro", currPos);
            telemetry.update();
        }
        stopMotors();
    }

    public double getGyroYaw(){
        double currPos;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        return currPos;
    }

    public double setTurn(double deg) { //Turns to a desired angle using the IMU in teleop.
        double turnPow = 0;
        double error;
        double errorMove;
        double pd = .0055;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)); // currPos is the current position

        if ((currPos > deg + 1 || currPos < deg - 1) && opModeIsActive()) { //While sensor isn't in the desired angle position, run.
            error = deg - currPos; //Finding how far away we are from the target position.
            errorMove = Math.abs(deg - currPos);
            if (error > 180) {
                error = error - 360;
            } else if (error < -180) {
                error = error + 360;
            }

            //The following code allows us to turn in the direction we need, and if we cross the axis
            //at which 180 degrees becomes -180, our robot can turn back in the direction which is closest
            //to the position we wish to be at (We won't make a full rotation to get to -175, if we hit 180).
            if (currPos < deg) {
                if (errorMove < 180) {
                    turnPow = - Math.abs(pd * error); //Turns left
                }
                if (errorMove > 180) {
                    turnPow = Math.abs(pd * error); //Turns right if we go past the pos/neg mark.
                }
            } else if (currPos > deg) {
                if (errorMove < 180) {
                    turnPow = Math.abs(pd * error); //Turns right
                }
                if (errorMove > 180) {
                    turnPow = - Math.abs(pd * error); //Turns left if we go past the pos/neg mark.
                }
            }

        }
        if (turnPow < 0 && -.075< turnPow) {
            turnPow = -.075;
        }
        else if (turnPow > 0 && .075 > turnPow) {
            turnPow = .075;
        }
        return turnPow;
    }

    public void setTurnAuto(double deg) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

        while (currPos > deg + 1 || currPos < deg - 1) { //while we are not at the right degree.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            setMotorsAll(0, 0, setTurn(deg));
            idle();
        }
    }



}
