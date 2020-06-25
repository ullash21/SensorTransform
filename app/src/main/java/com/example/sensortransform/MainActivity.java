package com.example.sensortransform;

import android.content.Context;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Environment;
import android.os.SystemClock;
import android.util.Log;
import android.view.Display;
import android.view.Surface;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;


import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import org.opencv.android.CameraActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoWriter;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;


//import androidx.camera.core.CameraX;
//import androidx.camera.core.ImageCapture;
////import androidx.camera.core.ImageCaptureConfig;
//import androidx.camera.core.Preview;
////import androidx.camera.core.PreviewConfig;
//import androidx.camera.core.impl.ImageCaptureConfig;
//import androidx.camera.core.impl.PreviewConfig;

/*
 * To dos
 * make factor, shortest time interval for all variables
 * UI design for start and stop measurement procedures
 * Go through enumerate in java
 * See if can get name or value info from Sensor API
 * Synchronise through flags or time intervals
 * go through sensorSurvey app to understand how sensor infos like names are collected
 * Understand UI implementation of Sensor survey app
 * */
public class MainActivity extends CameraActivity implements SensorEventListener, LocationListener , CvCameraViewListener2 {
    private static final int REQUEST_CODE_ASK_PERMISSIONS = 1;
    private static final int MY_PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION = 2;


    // Sensors
    private Sensor sensors[];
    private SensorManager mSensorManager;

    //GPS
    protected LocationManager locationManager;
    protected LocationListener locationListener;
    // TextViews to display current sensor values
    File file;
    File file2;
    File dir;
    File sdCard;
    File text;
    FileOutputStream f;
    FileOutputStream f2;
    String disData="";
    Mat frame;
    //Display
    private Display mDisplay;
    boolean cameraInintialised=false;
    boolean frameReady=false;
    boolean frameReady2=false;
    boolean gpsUpdated=false;
    //Sensor value type handler variables
    private String headers[] = {"TimeStamp", "AccX", "AccY", "AccZ", "GyX", "GyY", "GyZ", "Pressure", "GravX",
            "GravY", "GravZ", "linAccX", "linAccY", "linAccZ", "RotVecX", "RotVecY", "RotVecZ", "RotVecAngle",
            "RotVecAccuracy", "GameRotVectorX", "GameRotVectorY", "GameRotVectorZ",
            "GameRotVectorAngle", "GyUncalAngSpeedX", "GyUncalAngSpeedY",
            "GyUncalAngSpeedZ", "GyUncaldriftX", "GyUncaldriftY", "GyUncaldriftZ", "PoseX", "PoseY", "PoseZ",
            "PoseAngle", "PoseTransX", "PoseTransY", " PoseTransZ", "PoseDelRotX", "PoseDelRotY", "PoseDelRotZ",
            "PoseDelRotAngle", "PoseDelTransX", "PoseDelTransY", "PoseDelTransZ", "PoseSeqNum",
            "AccUncalX", "AccUncalY", "AccUncalZ", " AccUncalBX", "AccUncalBY", "AccUncalBZ", "magFieldX",
            "magFieldY", "magFieldZ", "magFieldUncalX", "magFieldUncalY", "magFieldUncalZ",
            "magFieldUncalBX", "magFieldUncalBY", "magFieldUncalBZ", "Latitude", "Longtitude"};

    private int types[] = {
            Sensor.TYPE_ACCELEROMETER,
            Sensor.TYPE_GYROSCOPE,
            Sensor.TYPE_PRESSURE,
            Sensor.TYPE_GRAVITY,
            Sensor.TYPE_LINEAR_ACCELERATION,
            Sensor.TYPE_ROTATION_VECTOR,
            Sensor.TYPE_GAME_ROTATION_VECTOR,
            Sensor.TYPE_GYROSCOPE_UNCALIBRATED,
            Sensor.TYPE_POSE_6DOF,
            Sensor.TYPE_ACCELEROMETER_UNCALIBRATED,
            Sensor.TYPE_MAGNETIC_FIELD,
            Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED
    };
    int valueLengths[] = {3, 3, 1, 3, 3, 5, 4, 6, 15, 6, 3, 6};// order of sensor values with types start except TimeStamp
    boolean headerWritten = false;
    boolean headerWritten2=false;
    Button button;
    VideoWriter videoWriter;
    VideoWriter videoWriter2;
    private CameraBridgeViewBase mOpenCvCameraView;
    private float[] rotat=new float[0];
    private String time;


    public MainActivity() {
    }


    private int REQUEST_CODE_PERMISSIONS = 101;
    private final String[] REQUIRED_PERMISSIONS = new String[]{"android.permission.CAMERA", "android.permission.WRITE_EXTERNAL_STORAGE",
            "android.permission.ACCESS_FINE_LOCATION", "android.permission.ACCESS_COARSE_LOCATION"};

    public void write(Mat frame,VideoWriter videoWriter) {
        if(videoWriter!=null&&videoWriter.isOpened()==false){
            videoWriter.release();
            throw new IllegalArgumentException("Video Writer Exception: VideoWriter not opened,"
                    + "check parameters.");
        }
        //Write video
        videoWriter.write(frame);
    }



    int l=0;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.v("on_create",""+l++);
        if (allPermissionsGranted()) {
            startSequence(); //start camera if permission has been granted by user
        } else {
            ActivityCompat.requestPermissions(this, REQUIRED_PERMISSIONS, REQUEST_CODE_PERMISSIONS);
        }
    }
    int o=0;
    //@SuppressLint({"MissingPermission", "SourceLockedOrientationActivity"})
    protected void startSequence() {
        Log.v("startSequence",""+o++);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        setContentView(R.layout.activity_main);
        button = findViewById(R.id.button);
        Log.v("oncreate","OPencvLoader:"+ OpenCVLoader.initDebug());
        boolean start = false;
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.CameraView);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (button.getText().equals("Start"))
                    button.setText("Stop");
                else
                    button.setText("Start");
                // Code here executes on main thread after user presses button
            }
        });

        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        sensors = new Sensor[12];


        WindowManager wm = (WindowManager) getSystemService(WINDOW_SERVICE);
        mDisplay = wm.getDefaultDisplay();

        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, this);

        int headerIterator=1;
        //String disData="";
        for(int i=0;i<sensors.length;i++) {
            sensors[i] = mSensorManager.getDefaultSensor(types[i]);

            String header[]=new String[valueLengths[i]];
            for(int j=0;j<valueLengths[i];j++) {
                header[j] = headers[headerIterator+j];
            }
            headerIterator+=valueLengths[i];
            if(sensors[i]!=null) {
                if(!MySensor.checkExistence(sensors[i].getType()))
                    MySensor.mySensors.add(new MySensor(sensors[i], header));
                disData+=sensors[i].getName()+":\n";

            }
        }
        //disData+="GPS:";
        Log.v("onstartsequence","success");
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        Log.v("onSensorChanged","check 1");
        if(event.accuracy!=SensorManager.SENSOR_STATUS_ACCURACY_LOW&&button.getText().equals("Stop")&&cameraInintialised) {
        //if(event.accuracy!=SensorManager.SENSOR_STATUS_ACCURACY_LOW&&cameraInintialised) {
            //Log.v("onSensorChanged","check 2");
            long unixTime=System.currentTimeMillis()*1000000 + (event.timestamp - SystemClock.elapsedRealtimeNanos());
            //String csvData = transformCurrentValue(event.values, event.sensor.getType(), event.timestamp);
            MySensor mySensor=MySensor.getSensor(event.sensor.getType());
            mySensor.update(event.values.clone());
            //Log.v("onSensorChanged","check 3");
            if(MySensor.writeReady()) {
                //Log.v("onSensorChanged","check 4");
                mySensor.setUpdateTime(event.timestamp);
                //long time=System.nanoTime();
                MySensor accelerometer=MySensor.getSensor(Sensor.TYPE_ACCELEROMETER);
                MySensor magnetometer=MySensor.getSensor(Sensor.TYPE_MAGNETIC_FIELD);
                float[][] orientationAndRotation=MySensor.getOrientation(mDisplay.getRotation(),accelerometer.getValuesWithoutCheck(),magnetometer.getValuesWithoutCheck());
                String[] transformedData=transformCurrentValue(unixTime,orientationAndRotation[0],orientationAndRotation[1]);
                csvWrite(transformedData[0]);
                //time=System.nanoTime()-time;
                //Log.i("write time",time+"");
                //mTextSensors.setText(transformedData[1]);
                //Log.v("onSensorChanged","check 5");
            }
        }
    }
    private String getHeader(){
        String gpsHeader=MyGPS.getHeader();
        String data="";
        data+="Timestamp,";
        for(MySensor mySensor:MySensor.mySensors)
            for (String s : mySensor.getHeader())
                data += s + ",";
        data+="azimuth"+","+"pitch"+","+"roll"+",";
        for(int i=0;i<rotat.length;i++){
            data+="rotation-"+i+",";
        }
        data+=gpsHeader;
        return data;
    }
    private String[] transformCurrentValue(long timestamp,float[] orientation,float[] rotation){
        String data="";
        disData="";
        rotat=rotation;
        String gpsHeader=MyGPS.getHeader();
        data+=timestamp+",";
        for(MySensor mySensor:MySensor.mySensors) {
            //disData+=mySensor.getName()+":\n";
            for (Float value : mySensor.getValues()) {
                data += value + ",";
                //disData+=value+" ,";
            }
            //disData=disData.substring(0,disData.length()-1)+"\n";
        }
        data+=""+orientation[0]+","+orientation[1]+","+orientation[2]+",";
        for(float i:rotation){
            data+=i+",";
        }
        data+=MyGPS.getValues();
        //disData+="orientation:"+orientation[0]+","+orientation[1]+","+orientation[2]+"\n";
        String[] headers=gpsHeader.substring(0,gpsHeader.length()-1).split(",");
//        for(int i=0;i<headers.length;i++) {
//            disData+=headers[i]+": "+ MyGPS.getValues().split(",")[i]+"\n";
//        }
        disData+=MySensor.getRate()+MyGPS.getRate();

        Log.d("DATA ", data);
        return new String[] {data,disData};
    }
    //int counter=0;
    private void csvWrite(String s){
        //if(counter<100) {
        try {
            String st="";
            if(headerWritten==false){
                st=getHeader();
                headerWritten=true;
            }
            f.write((st+s).getBytes());
            f.flush();
            if(gpsUpdated) {
                String st2="";
                if(headerWritten2==false){
                    st2=getHeader();
                    headerWritten2=true;
                }
                f2.write((st2+s).getBytes());
                f2.flush();
                gpsUpdated=false;
                frameReady2=true;
            }
            //            if(videoWriter!=null&&frame!=null) {
            //                Log.v(TAG,"width of mat:"+frame.size().width);
            //                Log.v(TAG,"height of mat:"+frame.size().height);
            //                Log.v(TAG,"cols:"+frame.channels());
            //                write(frame);
            //            }
            frameReady = true;
            //Log.i("camera_write_promt:",""+counter);
            //counter++;
            //Toast.makeText(getBaseContext(), "Data saved", Toast.LENGTH_LONG).show();
        } catch (IOException e) {
            e.printStackTrace();
        }
        //}
    }


    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {

        if(requestCode == REQUEST_CODE_PERMISSIONS){
            if(allPermissionsGranted()){
                startSequence();
                onStart();
                onCameraPermissionGranted();
            } else{
                Toast.makeText(this, "Permissions not granted by the user.", Toast.LENGTH_SHORT).show();
                finish();
            }
        }
    }

    private boolean allPermissionsGranted(){

        for(String permission : REQUIRED_PERMISSIONS){
            if(ContextCompat.checkSelfPermission(this, permission) != PackageManager.PERMISSION_GRANTED){
                return false;
            }
        }
        return true;
    }


    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
    @Override
    protected void onStart() {
        super.onStart();

        if (allPermissionsGranted()) {
            for(int i=0;i<sensors.length;i++)
                if (sensors[i] != null)
                    mSensorManager.registerListener(this, sensors[i],
                            SensorManager.SENSOR_DELAY_NORMAL);
            try {
                time =new SimpleDateFormat("yyyy'-'MM'-'dd'-'HH'-'mm'-'ss")
                        .format(new Date(System.currentTimeMillis()));
                sdCard = getExternalFilesDir (Environment.DIRECTORY_DCIM);

                dir = new File( sdCard.getAbsolutePath()+ "/"+time);
                if(!dir.exists()) {
                    Log.v("dir", dir.mkdir()+"");
                }
                Log.v("path",sdCard.getAbsolutePath() +"/"+time);
                file = new File(dir, "data-sensor.csv");
                file2 = new File(dir, "data-GPS.csv");
                text=new File(dir,"text.txt");
                f = new FileOutputStream(file, true);
                f2 = new FileOutputStream(file2, true);

                headerWritten=false;
                headerWritten2=false;
                Log.v("case 9","here");
                //imgDir = new File(Environment.getExternalStorageDirectory() + "/img/bpl");
                //boolean suc=imgDir.mkdir()
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }

        }

    }

    @Override
    public void onResume() {
        Log.v("onresume", "on resume");
        super.onResume();
        System.loadLibrary("opencv_java4");
        mOpenCvCameraView.enableView();
    }

    @Override
    protected void onStop() {
        super.onStop();
        Log.i("onstop","inside");
        try {
            if(f!=null)
                f.close();
            if(f2!=null)
                f2.close();
            FileOutputStream f2=new FileOutputStream(text,false);
            f2.write(disData.getBytes());
            f2.close();
            Log.i("camera_onstop","here");
            if(videoWriter!=null&&videoWriter.isOpened()) {
                videoWriter.release();
                Log.i("camera_cLOSED","success ");
                videoWriter=null;
            }
            if(videoWriter2!=null&&videoWriter2.isOpened()) {
                videoWriter2.release();
                Log.i("camera_cLOSED","success ");
                videoWriter2=null;
            }
        } catch (IOException e) {
            Log.i("error on Stop",e.toString());
        }
        mSensorManager.unregisterListener(this);
    }
    @Override
    protected void onDestroy() {
        Log.i("onDestroy","inside");
        super.onDestroy();
    }

    @Override
    public void onLocationChanged(Location location) {
        //Log.i("in Location changed",location.getLatitude()+"  "+location.getLongitude());
        new MyGPS(location);
        gpsUpdated=true;
    }

    @Override
    public void onProviderDisabled(String provider) {
        Log.d("Latitude","disable");
    }

    @Override
    public void onProviderEnabled(String provider) {
        Log.d("Latitude","enable");
    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {
        Log.d("Latitude","status");
    }
    String TAG="Camera";


    @Override
    protected List<? extends CameraBridgeViewBase> getCameraViewList() {
        return Collections.singletonList(mOpenCvCameraView);
    }
    @Override
    public void onCameraViewStarted(int width, int height) {

        Log.v(TAG, "in cam started");
       //Log.v(TAG,"width: "+width);
        //Log.v(TAG,"height: "+height);
        Log.i("camera_started","in");

        time =new SimpleDateFormat("yyyy'-'MM'-'dd'-'HH'-'mm'-'ss")
                .format(new Date(System.currentTimeMillis()));
        if(videoWriter==null)
            videoWriter =new VideoWriter(dir+"/"+time+"video-sensor.avi",  VideoWriter.fourcc('M', 'J', 'P', 'G'), 5,
                    new Size(width,height));
        if(videoWriter2==null)
            videoWriter2 =new VideoWriter(dir+"/"+time+"video-GPS.avi",  VideoWriter.fourcc('M', 'J', 'P', 'G'), 5,
                    new Size(width,height));
//        videoWriter.open(dir+"/video.avi", VideoWriter.fourcc('M','J','P','G'),
//                5.0D, new Size(width,height));
//        Log.v(TAG,"Videowriter: "+videoWriter.isOpened());
//        Log.v(TAG,"Videowriter: "+videoWriter2.isOpened());
//        Log.v(TAG,"videowriter height: "+mOpenCvCameraView.getHeight());
//        Log.v(TAG,"viderowriter width: "+mOpenCvCameraView.getWidth());
    }

    @Override
    public void onCameraViewStopped() {}
    int cameraframeint=0;
    //int cameraConter=0;
    long time2;
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        //Log.i("camera_counter ",cameraConter+"");
        frame = inputFrame.rgba();
        if(cameraframeint==0) {
//            Log.v(TAG, "in cam stopped");
//            Log.v(TAG, "MAt height: " + frame.size().height);
//            Log.v(TAG, "MAt width: " + frame.size().width);
//            Log.v(TAG,"width of mat:"+frame.size().width);
//            Log.v(TAG,"height of mat:"+frame.size().height);
//            Log.v(TAG,"cols:"+frame.channels());
            //Imgproc.putText(frame, cameraConter+"", new Point(10, 50), 3, 1, new Scalar(255, 0, 0, 255), 2);
            write(frame,videoWriter);
            write(frame,videoWriter2);
            cameraframeint++;
            cameraInintialised=true;
        }
        if(frameReady) {
            //Imgproc.putText(frame, cameraConter+"", new Point(10, 50), 3, 1, new Scalar(255, 0, 0, 255), 2);
            time2= System.currentTimeMillis();
            write(frame,videoWriter);
            Log.i("camera_write_sensor",System.currentTimeMillis()-time2+"");

            if(frameReady2){
                time2= System.currentTimeMillis();
                write(frame,videoWriter2);
                Log.i("camera_write_GPS",System.currentTimeMillis()-time2+"");
                frameReady2=false;
            }
            //Log.i("camera_counter",cameraConter+"");
            //cameraConter++;
        }
        frameReady=false;
        //Mat gray = new Mat();
        //Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGBA2GRAY);
        return frame;
    }
}

/************************************************************************************/
class MySensor{

    //A data structure to hold up and access sensor information
    /*Todo: parse string to get name automatically*/


    private final int type;
    private volatile float values[];
    private String[] header;
    private volatile boolean stale;// determined by if the value is fetched or not
    private static long updateTime=0;//latest time when the sensor was updated
    private boolean pseudo;// whether hardware or software sensor
    private long shortestInterval;
    private static long longestShortestInterval=0;
    private String name;
    private Sensor sensor;
    private static double maxRate=0;
    private static double minRate=0;
    private static double avgRate=0;
    private static long count=0;
    public static ArrayList<MySensor> mySensors=new ArrayList();

    //private static double latitude=0;
    //private static double longitude =0;
    public MySensor(Sensor sensor,String[] header) {
        this.sensor=sensor;
        this.type = sensor.getType();
        name=sensor.getName();
        pseudo=checkPseudo(name);
        this.header = header;
        stale=true;
        shortestInterval=sensor.getMinDelay();
        if(shortestInterval>longestShortestInterval)
            longestShortestInterval=shortestInterval;
        String h=".\n";
        for(String s: header)
            h+=s+"\n";
        //Log.i(sensor.getName(),h);
    }

    public static boolean checkExistence(int type) {
        if(getSensor(type)==null)
            return false;
        else
            return true;
    }

    String getName(){
        return name;
    }
    private static boolean checkPseudo(String str){
        String string=str.split(" ")[0];
        //convert String to char array
        char[] charArray = string.toCharArray();

        for(int i=0; i < charArray.length; i++){

            //if the character is a letter
            if( Character.isLetter(charArray[i]) ){

                //if any character is not in upper case, return false
                if( !Character.isUpperCase( charArray[i] ))
                    return true;
            }
        }

        return false;
    }
    public static boolean writeReady(){
        int ready=1;
        for(int i=0;i<mySensors.size();i++){
            ready *= mySensors.get(i).isStale() ? 0 : 1;
            //Log.v("write ready: "+mySensors.get(i).name,!mySensors.get(i).isStale()+"");
//            if(mySensors.get(i).isStale())
//                Log.v("write ready: "+mySensors.get(i).name,!mySensors.get(i).isStale()+"");
//            else
//                Log.v("write ready: "+mySensors.get(i).name,!mySensors.get(i).isStale()+"");
        }
        //Log.v("write ready",ready+"");
        return ready==1;
    }
    public static MySensor getSensor(int type){
        for(int i=0;i<mySensors.size();i++)
            if(type==mySensors.get(i).type)
                return mySensors.get(i);
        return null;
    }
    public boolean isPseudo() {
        return pseudo;
    }
    public int getLength(){
        return header.length;
    }

    public void update(float[] values){

        stale=false;
        //Log.v("update",name);
        //Log.v("update","stale value: "+stale);
        this.values=values;
    }
    private static void setRate(long difference){
        count++;
        double newRate=Math.pow(10,9)/difference;
        //Log.i("rate",newRate+"");
        //Log.i("rate count",count+"");
        //Log.i("rate difference",difference+"");
        if(minRate==0)
            minRate=newRate;
        else if (minRate>newRate)
            minRate=newRate;
        if(maxRate==0)
            maxRate=newRate;
        else if(maxRate<newRate)
            maxRate=newRate;
        if(avgRate==0)
            avgRate=newRate;
        else
            avgRate=avgRate*((count-1)*1.0/count)+newRate/count;
    }
    public static String getRate(){
        return "Sensors:\n"+"Max: "+maxRate+"\n"+
                "Min: "+minRate+"\n"+
                "Avg: "+avgRate+"\n";
    }
    public static void setUpdateTime(long timeStamp){
        if(timeStamp>updateTime)
            setRate(timeStamp-updateTime);
        updateTime=timeStamp;
    }

    public long getShortestInterval() {
        return shortestInterval;
    }

    public double getFrequency() {//Max frequency for sensor
        return Math.pow(10,9)/shortestInterval;
    }

    public String[] getHeader(){
        return this.header;
    }

    public int getType() {
        return type;
    }

    public float[] getValues() {
        //Log.v("updated"," value accessed:"+name);
        stale=true;
        return values;
    }
    public float[] getValuesWithoutCheck() {
        return values;
    }
    public boolean isStale() {
        return stale;
    }

    public static float[][] getOrientation(int rotaion, float[] mAccelerometerData, float[] mMagnetometerData) {
        //return new float[0];
        // Compute the rotation matrix: merges and translates the data
        // from the accelerometer and magnetometer, in the device coordinate
        // system, into a matrix in the world's coordinate system.
        //
        // The second argument is an inclination matrix, which isn't
        // used in this example.
        float[] rotationMatrix = new float[9];
        boolean rotationOK = SensorManager.getRotationMatrix(rotationMatrix,
                null, mAccelerometerData, mMagnetometerData);

        // Remap the matrix based on current device/activity rotation.
        float[] rotationMatrixAdjusted = new float[9];
        switch (rotaion) {
            case Surface.ROTATION_0:
                rotationMatrixAdjusted = rotationMatrix.clone();
                break;
            case Surface.ROTATION_90:
                SensorManager.remapCoordinateSystem(rotationMatrix,
                        SensorManager.AXIS_Y, SensorManager.AXIS_MINUS_X,
                        rotationMatrixAdjusted);
                break;
            case Surface.ROTATION_180:
                SensorManager.remapCoordinateSystem(rotationMatrix,
                        SensorManager.AXIS_MINUS_X, SensorManager.AXIS_MINUS_Y,
                        rotationMatrixAdjusted);
                break;
            case Surface.ROTATION_270:
                SensorManager.remapCoordinateSystem(rotationMatrix,
                        SensorManager.AXIS_MINUS_Y, SensorManager.AXIS_X,
                        rotationMatrixAdjusted);
                break;
        }

        // Get the orientation of the device (azimuth, pitch, roll) based
        // on the rotation matrix. Output units are radians.
        float orientationValues[] = new float[3];
        float rotationMatrixReturn[]=rotationMatrixAdjusted.clone();
        if (rotationOK) {
            SensorManager.getOrientation(rotationMatrixAdjusted,
                    orientationValues);
        }
        return new float[][]{orientationValues, rotationMatrixReturn};
    }
}
class MyGPS{
    private static double latitude=0;
    private static float bearing=0;
    private static float accuracy=0;
    private static double altitude=0;
    private static float bearingAccuracyDegrees=0;
    private static long elapsedRealTime = 0;
    private static float speed=0;
    private static float speedAccuracy=0;
    private static long gpsTime=0;
    private static double longitude =0;
    private static double maxRate=0;
    private static double minRate=0;
    private static double avgRate=0;
    private static long count=0;
    private static long updateTime=0;//latest time when the GPS was updated
    private static long elapsedRealTimePast=0;//latest time when the GPS was updated
    private static double carryLat=0;
    private static double oldLat=0;
    private static double carryLong=0;
    private static double oldLong=0;
    private static float distanceBetween=0;
    private static float bearingStart=0;
    private static float bearingEnd=0;
    MyGPS(@org.jetbrains.annotations.NotNull Location location){
        try{
            oldLong=carryLong;
            oldLat=carryLat;
            latitude=location.getLatitude();
            longitude=location.getLongitude();
            carryLat=latitude;
            carryLong=longitude;
            accuracy=location.getAccuracy();
            altitude=location.getAltitude();
            bearing=location.getBearing();
            bearingAccuracyDegrees=location.getBearingAccuracyDegrees();
            elapsedRealTime = location.getElapsedRealtimeNanos();
            gpsTime=location.getTime();
            speed=location.getSpeed();
            speedAccuracy=location.getSpeedAccuracyMetersPerSecond();
            updateTime=System.currentTimeMillis()*1000000L + (elapsedRealTime - SystemClock.elapsedRealtimeNanos());
            float[] x=new float[3];
            Location.distanceBetween(oldLat, oldLong,latitude,longitude,x);
            distanceBetween=x[0];
            bearingStart=x[1];
            bearingEnd=x[2];
            //Log.v("updateTime",updateTime+"");
            //Log.v("elapsedrealtime",elapsedRealTime+"");
            //if(location.getTime()>updateTime)

            setRate(elapsedRealTime-elapsedRealTimePast);
            //Log.v("diff",elapsedRealTime-elapsedRealTimePast+"");
            elapsedRealTimePast=elapsedRealTime;

        }
        catch (Exception e){
            Log.i("error loc init",e.toString());
        }
    }
    private static void setRate(long difference){
        if(difference!=0) {
            count++;
            //Log.v("count", count + "");
            double newRate = Math.pow(10, 9) / difference;
            //Log.v("newRate", newRate + "");
            if (minRate == 0)
                minRate = newRate;
            else if (minRate > newRate)
                minRate = newRate;
            if (maxRate == 0)
                maxRate = newRate;
            else if (maxRate < newRate)
                maxRate = newRate;
            if (avgRate == 0)
                avgRate = newRate;
            else
                avgRate = avgRate * ((count - 1) * 1.0 / count) + newRate / count;
        }
    }
    public static String getRate(){
        return "GPS:\n"+"Max: "+maxRate+"\n"+
                "Min: "+minRate+"\n"+
                "Avg: "+avgRate+"\n";
    }
    public static void setUpdateTime(long timeStamp){
    }
    public static String getValues(){
        return updateTime+","+ latitude+","+ longitude+","+ accuracy+","+ altitude+","+ bearing+","+ bearingAccuracyDegrees+","+
                elapsedRealTime+","+ speed+","+ speedAccuracy+","
                +distanceBetween+","+bearingStart+","+bearingEnd+"\n";
    }
    public static String getHeader(){

        return "gpsTime,latitude,longitude,accuracy,altitude,bearing,bearingAccuracyDegrees,elapsedrealTime," +
                "speed,speedAccuracy,distanceBetween,bearingStart,bearingEnd"+"\n";
    }
    public static double getLatitude() {
        return latitude;
    }

    public static void setLatitude(double latitude) {
        MyGPS.latitude = latitude;
    }

    public static float getBearing() {
        return bearing;
    }

    public static void setBearing(float bearing) {
        MyGPS.bearing = bearing;
    }

    public static float getAccuracy() {
        return accuracy;
    }

    public static void setAccuracy(float accuracy) {
        MyGPS.accuracy = accuracy;
    }

    public static double getAltitude() {
        return altitude;
    }

    public static void setAltitude(double altitude) {
        MyGPS.altitude = altitude;
    }

    public static float getBearingAccuracyDegrees() {
        return bearingAccuracyDegrees;
    }

    public static void setBearingAccuracyDegrees(float bearingAccuracyDegrees) {
        MyGPS.bearingAccuracyDegrees = bearingAccuracyDegrees;
    }

    public static long getElapsedrealTime() {
        return elapsedRealTime;
    }

    public static void setElapsedrealTime(long elapsedrealTime) {
        MyGPS.elapsedRealTime = elapsedrealTime;
    }

    public static float getSpeed() {
        return speed;
    }

    public static void setSpeed(float speed) {
        MyGPS.speed = speed;
    }

    public static float getSpeedAccuracy() {
        return speedAccuracy;
    }

    public static void setSpeedAccuracy(float speedAccuracy) {
        MyGPS.speedAccuracy = speedAccuracy;
    }

    public static long getLocationTime() {
        return gpsTime;
    }

    public static void setLocationTime(long locationTime) {
        MyGPS.gpsTime = locationTime;
    }

    public static double getLongitude() {
        return longitude;
    }

    public static void setLongitude(double longitude) {
        MyGPS.longitude = longitude;
    }
}