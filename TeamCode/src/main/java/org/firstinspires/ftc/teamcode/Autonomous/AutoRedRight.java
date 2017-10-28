///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
//import org.firstinspires.ftc.teamcode.Library.MyOpMode;
//
//
///**
// * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
// * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
// * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all linear OpModes contain.
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//
//@Autonomous(name="RedRight", group="Linear Opmode")
//
//public class AutoRedRight extends MyOpMode {
//
//
//    double servoArmD = 0.0;
//    double servoArmS = 0.0;
//    double servoHandL = 0.0;
//    double servoHandR = 0.0;
//    double servoHandS = 0.0;
//    double ultraDistance = 0.0;
//
//    boolean left = false;
//    boolean right = false;
//    boolean center = false;
//    boolean unknown = false;
//
//
//
//    OpenGLMatrix lastLocation = null;
//
//    VuforiaLocalizer vuforia;
//    VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//    VuforiaTrackable relicTemplate = relicTrackables.get(0);
//    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//
//    DcMotor motorBL;
//    DcMotor motorBR;
//    DcMotor motorFL;
//    DcMotor motorFR;
//
//    DcMotor liftLeft;
//    DcMotor liftRight;
//
//    CRServo jewelArm;
//    CRServo jewelHand;
//
//    ModernRoboticsI2cRangeSensor rightUltra;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//
//        hardwareMap();
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        parameters.vuforiaLicenseKey = "AebyfAz/////AAAAGQSk/SMIskOBiTwNytA2g40Z5EJHh/B+wOuXdcD3Am6MKfF5dFAXVTowhe3r4WWOLOXgM06SKRsGgwb/Wscw0JUVeut2HxkDwYkp/MXJcjzTLcr8Ss5QdCAUtyLX6x1QH+mp1fZ+k8CaVpYE2AgrLmclq4D6gCG5x0CVespmrQ4yGLHSsiiY8kxZAujvYcdXTldK3Utr6J7cL0EAgLSm590bcVaHkjIi3IZg9jX1168Ejz1q4B39gfL5aM6Icr4SyMbPG3cmPNko4Y3Ebf8OmzEanypRjKXGzbWAV237TJzu/wHcmWSEf8hRt1yZTAfZTUPkSwabx6qpyRSZdpK1lTnLLnGA/LqxM1N5oX/T1VG+";
//
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
//
//        telemetry.addData(">", "Press Play to start");
//        telemetry.update();
//        waitForStart();
//
//        relicTrackables.activate();
//
//
//
//        waitForStart();
//        runtime.reset();
//
//        while (opModeIsActive()) {
//
//            jewelKnockerRed(servoArmD, servoArmS, servoHandL, servoHandR, servoHandS);
//
//
//            if (vuMark == RelicRecoveryVuMark.CENTER) {
//                center = true;
//            }
//            else if (vuMark == RelicRecoveryVuMark.LEFT) {
//                left = true;
//            }
//            else if (vuMark == RelicRecoveryVuMark.RIGHT){
//                right = true;
//            }
//            else {
//                unknown = true;
//            }
//
//            //For if the jewel knocker is on the right side
////            moveTo(.25, 1000, .6, 2.2); //move 28in foward = move 2.2281692032 encoder ticks
//            mecAutoRight(.25, .25, 62, 5000); //hit wall is 23 in right = move 1.8302818455 encoder ticks
//            mecAutoLeft(.25, .25, 93, 7000); //move to center left = move 2.7852115040 encoder ticks
//            //move foward 20 in = 1.59154943092 encoder ticks
//
//            //Omni, add code for 2nd move to command into the if statement
////            moveTo(.25, 1000, .6, 2.2);
////            turnCorr(.25, -90, 2500);
////            moveTo(.25, 400, .6, 2.2);
////            turnCorr(.25, 90, 2500);
//
//
//
////            if (center = true) {
////                moveTo(.25,100, .6, 2.2);
////                depositBlockAuto(0.75);
////            }
////            else if (left = true) {
////                mecAutoLeft(.25, .25, 16, 2000);
////                moveTo(.25,100, .6, 2.2);
////                depositBlockAuto(0.75);
////            }
////            else if (right = true) {
////                mecAutoRight(.25, .25, 16, 2000);
////                moveTo(.25,100, .6, 2.2);
////                depositBlockAuto(0.75);
////            }
////            else if (unknown = true) {
////                moveTo(.25 ,100, .6, 2.2);
////                depositBlockAuto(0.75);
////            }
////end of auto
//
//        }
//        while (opModeIsActive()) {
//
//
//            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//
//                telemetry.addData("VuMark", "%s visible", vuMark);
//
//                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
//                telemetry.addData("Pose", format(pose));
//
//                if (pose != null) {
//                    VectorF trans = pose.getTranslation();
//                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//                    double tX = trans.get(0);
//                    double tY = trans.get(1);
//                    double tZ = trans.get(2);
//
//                    double rX = rot.firstAngle;
//                    double rY = rot.secondAngle;
//                    double rZ = rot.thirdAngle;
//                }
//            }
//            else {
//                telemetry.addData("VuMark", "not visible");
//            }
//
//            telemetry.update();
//        }
//    }
//
//    String format(OpenGLMatrix transformationMatrix) {
//        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
//    }
//
//}
//
