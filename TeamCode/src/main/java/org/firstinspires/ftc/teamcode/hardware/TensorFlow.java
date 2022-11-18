package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.subsystems.VuforiaKey;

import java.util.List;

public class TensorFlow {

    private final Telemetry _telemetry;
    private final String TFOD_MODEL_ASSET;// = "PowerPlay.tflite";
    private final String TFOD_MODEL_FILE;//  = null; //"/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    private final WebcamName camera;
    private final int tfodMonitorId;
    private static final String[] LABELS = {
            "1",
            "2",
            "3"
    };

    private static final String VUFORIA_KEY = VuforiaKey.Key;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public static class Parameters {
        int tfodmonitorid;
        Telemetry telemetry;
        WebcamName camera;
        String TFOD_MODEL_ASSET= "PowerPlay.tflite";
        String TFOD_MODEL_FILE=null;
    }

    public TensorFlow(TensorFlow.Parameters parameters) {
        this._telemetry = parameters.telemetry;
        this.TFOD_MODEL_FILE = parameters.TFOD_MODEL_FILE;
        this.TFOD_MODEL_ASSET = parameters.TFOD_MODEL_ASSET;
        this.tfodMonitorId = parameters.tfodmonitorid;
        this.camera = parameters.camera;

        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.1, 16.0/9.0);
        }
    }

    public String findBestMatch(int seconds) {
        ElapsedTime et = new ElapsedTime();
        et.reset();
        String maxLabel = null;
        double maxConfidence=-1;
        while (et.seconds() < 12 && maxLabel == null && maxConfidence < 0.8) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getConfidence() > maxConfidence) {
                        maxLabel = recognition.getLabel();
                        maxConfidence = recognition.getConfidence();
                    }
                } // after this loop, the most confident choice will be made
            }
        }
        return maxLabel;
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraName = camera;
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        TFObjectDetector.Parameters tfodParameters;
        if (tfodMonitorId < 0) {
            tfodParameters = new TFObjectDetector.Parameters();
        }
        else {
            tfodParameters = new TFObjectDetector.Parameters(tfodMonitorId);
        }
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        if (TFOD_MODEL_FILE != null) {
            tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        }
        else {
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        }
    }
}
