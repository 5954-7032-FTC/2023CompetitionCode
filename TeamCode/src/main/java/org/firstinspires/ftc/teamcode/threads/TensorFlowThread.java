package org.firstinspires.ftc.teamcode.threads;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import org.firstinspires.ftc.teamcode.RobotAutoDriveByGyro_Linear;
import org.firstinspires.ftc.teamcode.subsystems.VuforiaKey;

public class TensorFlowThread extends RobotThread {

    Telemetry _telemetry;
    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1",
            "2",
            "3"
    };

    private static final String VUFORIA_KEY = VuforiaKey.Key;
    /**
     * the variable we will use to store our instance of the Vuforia localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * the variable we will use to store our instance of the TensorFlow Object Detection engine.
     */
    private TFObjectDetector tfod;

    Telemetry.Item _T_image, _T_pos, _T_size, _T_count;
    RobotAutoDriveByGyro_Linear _top;
    public TensorFlowThread(int tfodmonitorid, Telemetry telemetry, WebcamName camera, RobotAutoDriveByGyro_Linear top) {
        TensorFlowInit(tfodmonitorid,telemetry,camera);
        _top = top;
    }

    public TensorFlowThread(int tfodmonitorid, Telemetry telemetry, WebcamName camera) {
        TensorFlowInit(tfodmonitorid,telemetry,camera);
    }

    private void TensorFlowInit(int tfodmonitorid, Telemetry telemetry, WebcamName camera) {
        this._telemetry = telemetry;
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia(camera);
        initTfod(tfodmonitorid);

        _T_count = _telemetry.addData("Count", 0);
        _T_image = _telemetry.addData("Image", 0);
        _T_pos = _telemetry.addData("- Position (Row/Col)", 0);
        _T_size =_telemetry.addData("- Size (Width/Height)", 0);

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }

    @Override
    public void run() {
        while (!isCancelled()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    _T_count.setValue( updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());


                        _T_image.setValue( "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        _T_pos.setValue( "%.0f / %.0f", row, col);
                        _T_size.setValue( "%.0f / %.0f", width, height);
                        _telemetry.update();
                        if (recognition.getConfidence() > 0.80) {
                            this.shutdown(recognition.getLabel());
                            break;
                        }
                    }
                    //_telemetry.update();
                }
            }
        }
    }

    public void shutdown(String s) {
        tfod.shutdown();
        vuforia.close();
        _top.setWhereToEnd(s);
        this.cancel();
        vuforia=null;
        tfod=null;

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(WebcamName cameraName) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.cameraName = cameraName;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(int monitorId) {
        TFObjectDetector.Parameters tfodParameters;
        if (monitorId < 0) {
            tfodParameters = new TFObjectDetector.Parameters();
        }
        else {
            tfodParameters = new TFObjectDetector.Parameters(monitorId);
        }
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);


        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
