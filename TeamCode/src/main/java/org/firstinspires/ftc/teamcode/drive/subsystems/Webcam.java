/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
import org.firstinspires.ftc.teamcode.drive.opmodes.Calibration;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.drive.subsystems.Util;
import org.firstinspires.ftc.teamcode.drive.subsystems.Lights;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class Webcam {

    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    public RobotBase robotBase;
    public String barcodeLocation;
    public String objectLabel = "";
    public double objectLeft = 0.0;
    public double objectTop = 0.0;
    public double objectRight = 0.0;
    public double objectBottom = 0.0;
    public float freightLocationLeft;
    public float freightLocationRight;
    protected Util util;
    protected Lights lights;
    private static final Integer webcamSamples = 5;
    public boolean initialized = false;

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String[] BARCODE_LOCATION = {
            "Right",
            "Center",
            "Left"
    };

    private static final float  minResultConfidence = 0.6f;

    /*
     * A Vuforia 'Development' license key can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     */
    private static final String VUFORIA_KEY =
            "ARXKY4r/////AAABmb9ScOj+l0t0ira7sOidsLBSfpaAW93Eice6VSZw+/6407uwU19CCmRcgiwVSM/jPNg0xidl+w2MSEiJIun4VopbwVTeTZ/g0drowWZfIqxZQyEjW+xE5+rZ5JFf9X5tAkRmEhVyGu5F1luxs1RlHh2VkNxu8h7aQV98izYxc4FmvKCqTPB/otF3ncjr1YgyQGKGT6WXeDAXb1FRxlBQ8eETyqJWLWswZOW5yLyXKb2ufZz+9JevXOzJIbAuECTdXgJiQolYQvGzY+zLM00hxRpKj7M7buHEqCFKque/yotyYCGlTX5RgNuGMEDKN2lIxhyR7dYvLwarfWmHRnyCR2C9CboRWy9pGgwCtQrf1gnj";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public Webcam(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;
        this.util = robotBase.util;
        this.lights = robotBase.lights;
        initHardware();
    }

    protected void initHardware() {
        initVuforia();
        initTfod();
    }


    public void getObjectDetection () {
        int sampleCount = 0;
        while (initialized && sampleCount <= webcamSamples) {
            sampleCount++;
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    //telemetry.addData("# Object Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        objectLabel = recognition.getLabel();
                        if (objectLabel == "Cube" || objectLabel == "Duck") {
                            objectLeft = recognition.getLeft();
                            objectTop = recognition.getTop();
                            objectRight = recognition.getRight();
                            objectBottom = recognition.getBottom();

                            //telemetry.addData(String.format("location (%d)", i), barcodeLocation);
                            //telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            //freightLocationLeft = recognition.getLeft();
                            //telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                            //freightLocationLeft = recognition.getRight();
                            //telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
                        }
                    }
                }
            } else {
                telemetry.addData("TFOD is null", "<<");
            }
            getBarcodeLocation(objectLeft, objectRight);
        }

    }

    private void getBarcodeLocation (double leftSide, double rightSide) {
//        if (rightSide <= 200) {
//            barcodeLocation = "Left";
//        } else if (rightSide >= 600)  {
//            barcodeLocation = "Right";
//        } else {
//            barcodeLocation = "Center";
//        }

        if (rightSide <= 310 && leftSide > 10)  {
            barcodeLocation = "Center";
        } else if (leftSide > 300) {
            barcodeLocation = "Right";
        } else {
            barcodeLocation = "Left";
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
            initialized = true;
        }
    }


    public void stop() {
        if (initialized && tfod != null) {
            tfod.shutdown();
            initialized = false;
        }
    }

    public void reset() {
    }
}
