package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.os.SystemClock;
import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import java.util.ArrayList;
import java.util.List;

import org.opencv.imgproc.Imgproc;

import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Point;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgproc.Moments;


import org.opencv.core.MatOfPoint;
import org.opencv.core.Point3;

import gov.nasa.arc.astrobee.Result;

import gov.nasa.arc.astrobee.types.Quaternion;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */


public class YourService extends KiboRpcService {
    int maxX,minX,maxY,minY;
    Result result;
    final int LOOP_MAX = 5;
    int loopCounter = 0;
    List<MatOfPoint3f> offset = new ArrayList<>();
    Rect AR_ROI;
    @Override
    protected void runPlan1() {
        ////////////////////////////////////////////////////////MISSION START//////////////////////////////////////////////////////////////
        api.startMission();
        ////////////////////////////////////////////////////////Log.i("startMission_status", "startMission succeeded" );

        //starting point
        result = moveToWrapper(10.76150f, -6.88490f, 5.31647f, 0f, 0f, -0.707f, 0.707f);

        /*while (!result.hasSucceeded() && loopCounter < LOOP_MAX) {
            result = moveToWrapper(10.76150f, -6.88490f, 5.31647f, 0f, 0f, -0.707f, 0.707f);
            ++loopCounter;
        }*/

        ////////////////////////////////////////////////////////Log.i("StartingPoint_status", "moved to starting position" );

////////////////////////////////////////////////////////TARGET1//////////////////////////////////////////////////////////////
        //moveTo point1
        //result = moveToWrapper(10.71000D, -7.70000D, 4.48000D, 0D, 0.707D, 0D, 0.707D);
        // report point1 arrival
        //api.reportPoint1Arrival();

        //Log.i("Target1_status", "moved to point 1 and reported point 1 arrival" );
        //api.saveMatImage(api.getMatNavCam(), "target1 at point1.jpeg");

        result = moveToWrapper(10.71000D, -7.75000D, 4.48000D, 0D, 0.707D, 0D, 0.707D);
        ////////////////////////////////////////////////////////Log.i("Target1_status", "moved to target1 laser shooting point" );
        // report point1 arrival
        api.reportPoint1Arrival();

        // irradiate the laser
        api.laserControl(true);
        ////////////////////////////////////////////////////////api.saveMatImage(api.getMatNavCam(), "target1 laser shot.jpeg");

        // take target1 snapshots
        api.takeTarget1Snapshot();

        // turn the laser off
        api.laserControl(false);

        ////////////////////////////////////////////////////////Log.i("Target1_status", "Target1 was shot, Start moving to point 2");

////////////////////////////////////////////////////////TARGET2//////////////////////////////////////////////////////////////

        //avoid KOZ (to be determined)
        result = moveToWrapper (11.2D,-9.511100D,4.48D,  0D, 0D, -0.707D, 0.707D);
        ////////////////////////////////////////////////////////Log.i("Target2_status", "Moved to first middle point to avoid KOZ");

        //move to point2
        //result = moveToWrapper(11.27460f, -9.92284f, 5.29881f, 0f, 0f, -0.707f, 0.707f);
        //Log.i("Target2_status", "Moved to point 2");
        //api.saveMatImage(api.getMatNavCam(), "target2 at point2.jpeg");

        result = moveToWrapper(11.18422807D, -10.1D, 5.485886301D, 0D, 0D, -0.707D, 0.707D);
        ////////////////////////////////////////////////////////Log.i("Target2_status", "Moved to target2 laser shooting position");

        try {
            Thread.sleep(5000);
        } catch (Exception e) {}

        Mat image = api.getMatNavCam();
        //Mat reserve= api.getMatNavCam();
        api.saveMatImage(image, "target2 at laser shooting position.jpeg");

        // irradiate the laser
        /*api.laserControl(true);
        Log.i("Target2_status", "Laser TEST SHOT");
        image = api.getMatNavCam();
        api.saveMatImage(image, "target2 LASER TEST.jpeg");
        // take target2 snapshot
        api.takeTarget2Snapshot();
        // turn the laser off
        api.laserControl(false);*/

        ////////////////////////////////////////////////////////VARIABLES//////////////////////////////////////////////////////////////
        double[][] camera_param = api.getNavCamIntrinsics();
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_64FC1);
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> markerCorners = new ArrayList<>();
        Mat markerIds = new Mat();
        DetectorParameters parameters = DetectorParameters.create();
        List<Point> markerCenterPoints = new ArrayList<>();
        Point centerOfAllCenters = new Point();
        Mat rvec = new Mat();
        Mat tvec = new Mat();
         // Mat b_rvec = new Mat();
        //Mat b_tvec = new Mat();
        cameraMatrix.put(0, 0, camera_param[0]);
        distCoeffs.put(0, 0, camera_param[1]);

        Mat source_cropped = detectArucoMarkers(image,cameraMatrix,distCoeffs,dictionary,markerCorners,markerIds,parameters,rvec, tvec, markerCenterPoints,centerOfAllCenters);

        Point center_in_pixels = findCircleCenter(source_cropped);
        ////////////////////////////////////////////////////////Log.i("Circle_detection", "circle center in cropped: ("+ center_in_pixels.x + ", " + center_in_pixels.y + ")");
        center_in_pixels.x = center_in_pixels.x + AR_ROI.x;
        center_in_pixels.y = center_in_pixels.y + AR_ROI.y;
        Log.i("Circle_detection", "circle center in original: ("+ center_in_pixels.x + ", " + center_in_pixels.y + ")");

        final double[] laser = {777, 440};
        final double[] euler_turningAngle = find_turningAngle(center_in_pixels.x , center_in_pixels.y , laser);
        ////////////////////////////////////////////////////////Log.i("angles ", "euler_turningAngle = ("+ euler_turningAngle[0] + ", " + euler_turningAngle[1] + ")");
        final Quaternion q_turningAngle = eulerToQuaternion(euler_turningAngle[1], 0, euler_turningAngle[0]); // z y x
        ////////////////////////////////////////////////////////Log.i("angles ", "q_turningAngle = ("+ q_turningAngle.getX() + ", " + q_turningAngle.getY() + ", " +q_turningAngle.getZ() + ", " +q_turningAngle.getW() + ") ");
        final Quaternion target_angle  = combineQuaternion(q_turningAngle, new Quaternion(0, 0, -0.707f, 0.707f));

        result = moveToWrapper(11.22481667D, -10.1D, 5.493206667D, target_angle.getX(), target_angle.getY(), target_angle.getZ(), target_angle.getW());

        //test_board(cameraMatrix, distCoeffs,  markerCorners,  markerIds, b_tvec,  b_rvec,  offset);

        //double[] target2_test_pos = calTargetPos();
        //Log.i("target2_test_pos", "target2_test_pos: ("+ target2_test_pos[0] + ", " + target2_test_pos[1] + ")");
        //Point3 target2 = find_target_cam(rvec, tvec);
        //alignLaser(target2);


        // take target2 snapshots
            // irradiate the laser
        api.laserControl(true);
        api.saveMatImage(api.getMatNavCam(), "target2 (1).jpeg");
        // take target2 snapshot
        api.takeTarget2Snapshot();
        // turn the laser off
        api.laserControl(false);

        ////////////////////////////////////////////////////////Log.i("Target2_status", "Laser shot N°" + snapCounter + "succeeded, continue");

        //realignLaser(image2);

        ////////////////////////////////////////////////////////Log.i("Target2_status", "All snapshots taken, moving to goal position");

////////////////////////////////////////////////////////GOAL//////////////////////////////////////////////////////////////
        //avoid KOZ
        result = moveToWrapper (10.71000D, -9.8111000D, 5.4615D,  0D, 0D, -0.707D, 0.707D);

        ////////////////////////////////////////////////////////Log.i("Goal_status", "Moved to middle point to avoid KOZ");

        //moving to goal position
        result = moveToWrapper(11.27460D, -7.89178D, 4.96538D, 0D, 0D, -0.707D, 0.707D);

        ////////////////////////////////////////////////////////Log.i("Goal_status", "Moved to goal");

        // send mission completion
        api.reportMissionCompletion();

/////////////////////////////////////////////test with ROI/////////////////////////////////////////////

        //predefined ROI
        /*Mat source_cropped = detectArucoMarkers(reserve,cameraMatrix,distCoeffs,dictionary,markerCorners,markerIds,parameters,rvec, tvec, markerCenterPoints,centerOfAllCenters );
        center_in_pixels = findCircleCenter(source_cropped);
        Log.i("Circle_detection", "circle center in cropped: ("+ center_in_pixels.x + ", " + center_in_pixels.y + ")");
        Log.i("Circle_detection", "circle center in cropped: ("+ center_in_pixels.x + ", " + center_in_pixels.y + ")");


        */

        //3) test with calculated ROI
        //Rect  AR_ROI = find_ROI2D(reserve, markerIds, markerCorners, markerCenterPoints, centerOfAllCenters);
        //Log.i("ARUCO test", "AR_ROI = ("+AR_ROI.x + ", " + AR_ROI.y + ", " + AR_ROI.width + ", " + AR_ROI.height +")");
        //Mat cropped_source = new Mat(reserve, AR_ROI);
        //api.saveMatImage(cropped_source, "with calculated ROI target2 AR_ROI from cropped_source.jpeg");

        /*Aruco.detectMarkers(cropped_source, dictionary, markerCorners, markerIds, parameters);
        Log.i("ARUCO TEST", "with calculated ROI markerIds.dump()" + markerIds.dump());
        for (Mat corner : markerCorners) {
            Log.i("ARUCO TEST", "with calculated ROI corner.dump()" + corner.dump());
        }
        //Mat outputImage = cropped_source.clone();
        //Aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds);
        //api.saveMatImage(outputImage, "with calculated ROI target2 drawdetectedmarkers.jpeg");
        //Aruco.estimatePoseSingleMarkers(markerCorners, 0.05f, cameraMatrix, distCoeffs, rvec, tvec);
        //Log.i("ARUCO test ", "with calculated ROI rvec.dump() = " + rvec.dump());
        //Log.i("ARUCO test ", "with calculated ROI tvec.dump() = " + tvec.dump());

        //4) test undistorted points
        List <List <Point>> all_points = new ArrayList<>();
        all_points.add(markerCornerPointsID11);
        all_points.add(markerCornerPointsID12);
        all_points.add(markerCornerPointsID13);
        all_points.add(markerCornerPointsID14);
        all_points.add(markerCenterPoints);
        for (List <Point> list : all_points) {
            for (Point point : list){
                point = undistort(reserve, cameraMatrix, distCoeffs, point);
                Log.i("undistorting points", "Point (" + point.x + ", " + point.y + ")");
            }
        }
        centerOfAllCenters = undistort(reserve, cameraMatrix, distCoeffs, centerOfAllCenters);
        Log.i("undistorting points", "undistorted centerOfAllCenters" + centerOfAllCenters.x + ", " + centerOfAllCenters.y + ")");
        find_ROI2D(reserve, markerIds, markerCorners, markerCenterPoints,centerOfAllCenters);
*/
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }
    private double[] find_turningAngle(double circlecenterx,double circlecentery, double[] laser) {
        final double xDistance = circlecenterx - laser[0];
        final double yDistance = laser[1] - circlecentery;
        ////////////////////////////////////////////////////////Log.i("find_turningAngle", "xDistance=" + xDistance + " ; yDistance=" + yDistance);

        final double anglePerPixel = 0.08125;
        //anglePerPixel = + 130 / Math.sqrt(Math.pow(1280, 2) + Math.pow(960, 2));

        double xAngle = xDistance * anglePerPixel;
        double yAngle = yDistance * anglePerPixel;
        ////////////////////////////////////////////////////////Log.i("find_turningAngle", "xAngle=" + xAngle + "yAngle=" + yAngle);
        return new double[]{xAngle, yAngle};
    }

    private Quaternion combineQuaternion(Quaternion newOrientation, Quaternion oldOrientation) {
        double x =  newOrientation.getX() * oldOrientation.getW() + newOrientation.getY() * oldOrientation.getZ()
                - newOrientation.getZ() * oldOrientation.getY() + newOrientation.getW() * oldOrientation.getX();
        double y = -newOrientation.getX() * oldOrientation.getZ() + newOrientation.getY() * oldOrientation.getW()
                + newOrientation.getZ() * oldOrientation.getX() + newOrientation.getW() * oldOrientation.getY();
        double z =  newOrientation.getX() * oldOrientation.getY() - newOrientation.getY() * oldOrientation.getX()
                + newOrientation.getZ() * oldOrientation.getW() + newOrientation.getW() * oldOrientation.getZ();
        double w = -newOrientation.getX() * oldOrientation.getX() - newOrientation.getY() * oldOrientation.getY()
                - newOrientation.getZ() * oldOrientation.getZ() + newOrientation.getW() * oldOrientation.getW();
        ////////////////////////////////////////////////////////Log.i("combineQuaternion", " x:" + x + " y:" + y + " z:" + z + " w:" + w);
        return new Quaternion((float)x, (float)y, (float)z, (float)w);
    }

    public double[] getCenter(List<Point> points) {
        final MatOfPoint points_ = new MatOfPoint();
        points_.fromList(points);
        return getCenter(points_);
    }

    public double[] getCenter(MatOfPoint points) {

        Moments moments = Imgproc.moments(points);
        return (new double[]{moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00()});
    }

    public Mat detectArucoMarkers(Mat source, Mat cameraMatrix, Mat distCoeffs, Dictionary dictionary, List<Mat> markerCorners, Mat markerIds,DetectorParameters parameters,Mat rvec, Mat tvec,List<Point> markerCenterPoints,Point centerOfAllCenters) {

        //=>>>>>>>>>detected ROI
        Aruco.detectMarkers(source, dictionary, markerCorners, markerIds, parameters);
        AR_ROI = find_ROI2D(source, markerIds, markerCorners, markerCenterPoints,centerOfAllCenters);

        //Rect AR_ROI = find_ROI2D_test(markerCorners, markerIds);
        ////////////////////////////////////////////////////////Log.i("ARUCO test", "AR_ROI = ("+AR_ROI.x + ", " + AR_ROI.y + ", " + AR_ROI.width + ", " + AR_ROI.height +")");
        Mat cropped_source = new Mat(source, AR_ROI);
        api.saveMatImage(cropped_source, "with calculated ROI target2 AR_ROI from cropped_source.jpeg");
        //Aruco.detectMarkers(cropped_source, dictionary, markerCorners, markerIds, parameters);
        ////////////////////////////////////////////////////////Log.i("ARUCO TEST", "with calculated ROI markerIds.dump()" + markerIds.dump());
        ////////////////////////////////////////////////////////for (Mat corner : markerCorners) {
        ////////////////////////////////////////////////////////    Log.i("ARUCO TEST", "with calculated ROI corner.dump()" + corner.dump());
        ////////////////////////////////////////////////////////}


        //2) test with predefined ROI
        //Rect AR_ROI = new Rect(545, 460, 340, 229);
        //Mat source_test = new Mat(source, AR_ROI);
        //api.saveMatImage(source_test, "with predefined ROI target2 AR_ROI from source_test.jpeg");

        //Aruco.detectMarkers(source_test, dictionary, markerCorners, markerIds, parameters);
       // Log.i("ARUCO TEST", "with predefined ROI markerIds.dump()" + markerIds.dump());
        //for (Mat corner : markerCorners) {
         //   Log.i("ARUCO TEST", "with predefined ROI corner.dump()" + corner.dump());
        //}

        //1) test without ROI
        //Aruco.detectMarkers(source, dictionary, markerCorners, markerIds, parameters);
        //Log.i("ARUCO TEST", "without ROI markerIds.dump()" + markerIds.dump());
        //for (Mat corner : markerCorners) {
        //    Log.i("ARUCO TEST", "without ROI corner.dump()" + corner.dump());
        //}
        //Mat outputImage = source.clone();
        //Aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds);
        //api.saveMatImage(outputImage, "without ROI target2 drawdetectedmarkers.jpeg");
        //Aruco.estimatePoseSingleMarkers(markerCorners, 0.05f, cameraMatrix, distCoeffs, rvec, tvec);
        //Log.i("ARUCO test ", "without ROI rvec.dump() = " + rvec.dump());
        //Log.i("ARUCO test ", "without ROI tvec.dump() = " + tvec.dump());



        //outputImage = source_test.clone();
        //Aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds);
        //api.saveMatImage(outputImage, "with predefined ROI target2 drawdetectedmarkers.jpeg");
        //Aruco.estimatePoseSingleMarkers(markerCorners, 0.05f, cameraMatrix, distCoeffs, rvec, tvec);
        //Log.i("ARUCO test ", "with predefined ROI rvec.dump() = " + rvec.dump());
        //Log.i("ARUCO test ", "with predefined ROI tvec.dump() = " + tvec.dump());

/*
        //3) test with calculated ROI
        AR_ROI = find_ROI2D(source, markerIds, markerCorners, markerCenterPoints, centerOfAllCenters);
        Log.i("ARUCO test", "AR_ROI = ("+AR_ROI.x + ", " + AR_ROI.y + ", " + AR_ROI.width + ", " + AR_ROI.height +")");
        Mat cropped_source = new Mat(api.getMatNavCam(), AR_ROI);
        api.saveMatImage(cropped_source, "with calculated ROI target2 AR_ROI from cropped_source.jpeg");

        Aruco.detectMarkers(cropped_source, dictionary, markerCorners, markerIds, parameters);
        Log.i("ARUCO TEST", "with calculated ROI markerIds.dump()" + markerIds.dump());
        for (Mat corner : markerCorners) {
            Log.i("ARUCO TEST", "with calculated ROI corner.dump()" + corner.dump());
        }
        outputImage = cropped_source.clone();
        Aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds);
        api.saveMatImage(outputImage, "with calculated ROI target2 drawdetectedmarkers.jpeg");
        Aruco.estimatePoseSingleMarkers(markerCorners, 0.05f, cameraMatrix, distCoeffs, rvec, tvec);
        Log.i("ARUCO test ", "with calculated ROI rvec.dump() = " + rvec.dump());
        Log.i("ARUCO test ", "with calculated ROI tvec.dump() = " + tvec.dump());*/

        //4) test undistorted points
        /*List <List <Point>> all_points = new ArrayList<>();
        all_points.add(markerCornerPointsID11);
        all_points.add(markerCornerPointsID12);
        all_points.add(markerCornerPointsID13);
        all_points.add(markerCornerPointsID14);
        all_points.add(markerCenterPoints);
        for (List <Point> list : all_points) {
            for (Point point : list){
                point = undistort(source, cameraMatrix, distCoeffs, point);
                Log.i("undistorting points", "Point (" + point.x + ", " + point.y + ")");
            }
        }*/
        //centerOfAllCenters = undistort(source, cameraMatrix, distCoeffs, centerOfAllCenters);
        //Log.i("undistorting points", "undistorted centerOfAllCenters" + centerOfAllCenters.x + ", " + centerOfAllCenters.y + ")");
        //find_ROI2D(source, markerIds, markerCorners, markerCenterPoints,centerOfAllCenters);

        //TBV
        //Point3 center_cam_frame = find_target_center_cam_frame(centerOfAllMarkers, rvec, tvec, id12);
        //Log.i("ARUCO test", "Center of markers in camera frame: ("+ center_cam_frame.x +", "+center_cam_frame.y+ ", " + center_cam_frame.z + ")");

        return cropped_source;
    }


    double[][] multiplyMat(double[][] firstMatrix, double[][] secondMatrix) {
        double[][] result = new double[firstMatrix.length][secondMatrix[0].length];

        for (int row = 0; row < result.length; row++) {
            for (int col = 0; col < result[row].length; col++) {
                for (int i = 0; i < secondMatrix.length; i++) {
                    result[row][col] += firstMatrix[row][i] * secondMatrix[i][col];
                }
            }
        }
        return result;
    }
    double length (Point3 p1, Point3 p2){
        return Math.sqrt(Math.pow(p1.x-p2.x,2)+Math.pow(p1.y-p2.y,2)+Math.pow(p1.z-p2.z,2));
    }
    double length (Point p1, Point p2){
        return Math.sqrt(Math.pow(p1.x-p2.x,2)+Math.pow(p1.y-p2.y,2));
    }
    double length (double p1, double p2){
        return Math.sqrt(Math.pow(p1,2)+Math.pow(p1,2));
    }


    private Quaternion eulerToQuaternion(double xAngle, double yAngle, double zAngle) {
        xAngle = Math.toRadians(xAngle);
        yAngle = Math.toRadians(yAngle);
        zAngle = Math.toRadians(zAngle);
        double c1 = Math.cos(yAngle/2);
        double s1 = Math.sin(yAngle/2);
        double c2 = Math.cos(zAngle/2);
        double s2 = Math.sin(zAngle/2);
        double c3 = Math.cos(xAngle/2);
        double s3 = Math.sin(xAngle/2);
        double w = c1*c2*c3 - s1*s2*s3;
        double x = s1*s2*c3 + c1*c2*s3;
        double y = s1*c2*c3 + c1*s2*s3;
        double z = c1*s2*c3 - s1*c2*s3;

        ////////////////////////////////////////////////////////Log.i("Convert euler angle to quaternion", " x:" + x + " y:" + y + " z:" + z + " w:" + w);
        return new Quaternion((float)x, (float) y, (float)z, (float)w);
    }

    private Result moveToWrapper(double pos_x, double pos_y, double pos_z,
                                 double qua_x, double qua_y, double qua_z,
                                 double qua_w) {
        //final Point point = new Point(pos_x, pos_y, pos_z);
        final gov.nasa.arc.astrobee.types.Point point = new gov.nasa.arc.astrobee.types.Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        return (api.moveTo(point, quaternion, true));
    }

    private Result relativeMoveToWrapper(double pos_x, double pos_y, double pos_z,
                                         double qua_x, double qua_y, double qua_z,
                                         double qua_w) {

        final gov.nasa.arc.astrobee.types.Point point = new gov.nasa.arc.astrobee.types.Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        return (api.relativeMoveTo(point, quaternion, true));
    }

    public Point findCircleCenter(Mat image){
        Point target_center=new Point();
        int potentialcercles=0;
        Mat src= image.clone();
        List<Point> circleCenters = new ArrayList<>();
        //Mat gray = new Mat();
        //Imgproc.cvtColor(src, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.medianBlur(src, src, 5);
        Mat circles = new Mat();
        Imgproc.HoughCircles(src, circles, Imgproc.HOUGH_GRADIENT, 1.0D,
                1D, // change this value to detect circles with different distances to each other
                100D, 41.0D, 1, 15); // change the last two parameters
        // (min_radius & max_radius) to detect larger circles
        for (int x = 0; x < circles.cols(); x++) {
            double[] c = circles.get(0, x);
            Point center = new Point(Math.round(c[0]), Math.round(c[1]));
            // circle center
            ////////////////////////////////////////////////////////Imgproc.circle(src, center, 1, new Scalar(100, 100, 0), 3, 8, 0 );
            // circle outline
            int radius = (int) Math.round(c[2]);
            ////////////////////////////////////////////////////////Imgproc.circle(src, center, radius, new Scalar(100, 100, 0), 3, 8, 0 );
            Log.i("Cercle_detection" , "cercle N°" + x + " = (" + center.x + ", " +center.y +") , radius = "+ radius);
            circleCenters.add(center);
            target_center.x += center.x;
            target_center.y += center.y;
            ////////////////////////////////////////////////////////Log.i("Cercle_detection"," target_center X : " + target_center.x  + " target_center.y  : " + target_center.y );
            potentialcercles++;
        }
        target_center.x=target_center.x/potentialcercles;
        target_center.y=target_center.y/potentialcercles;
        ////////////////////////////////////////////////////////Log.i("Cercle_detection","FINAL target_center X : " + target_center.x  + " target_center.y  : " + target_center.y );
        ////////////////////////////////////////////////////////api.saveMatImage(src, "target2 target center.jpeg");
        return target_center;
    }

    public Rect find_ROI2D(Mat source, Mat markerIds, List<Mat> markerCorners, List<Point> markerCenterPoints, Point centerOfAllCenters) {
        int i=-1;
        for (Mat mat : markerCorners) {
            i++;
            ////////////////////////////////////////////////////////List<Point> markerCornerPoints = new ArrayList<>();
            for (int row = 0; row < mat.height(); row++) {
                for (int col = 0; col < mat.width(); col++) {
                    final Point point = new Point(mat.get(row, col));
                    ////////////////////////////////////////////////////////Log.i("find_ROI2D", "new Point(mat.get(" + row + ", " + col + ")) = (" + point.x + ", " + point.y + ")");
                    ////////////////////////////////////////////////////////markerCornerPoints.add(point);
                    ////////////////////////////////////////////////////////Imgproc.circle(source, point, 3, new Scalar(100, 100, 0), -1);
                    ////////////////////////////////////////////////////////Log.i("find_ROI2D", "target2 markerCornerPoints");
                    if ((markerIds.get(i, 0)[0]==12) && (col==1)){
                        minX=(int) point.x;
                        minY=(int) point.y;
                        ////////////////////////////////////////////////////////Log.i("find_ROI2D ", "target2 minX = " + minX + "; minY = "+ minY);
                    }
                    else if ((markerIds.get(i, 0)[0]==14) && (col==3)){
                        maxX=(int) point.x;
                        maxY=(int) point.y;
                        ////////////////////////////////////////////////////////Log.i("find_ROI2D ", "target2 maxX = " + maxX + "; maxY = "+ maxY);
                    }
                }
            }
            ////////////////////////////////////////////////////////api.saveMatImage(source, "markerCornerPoints.jpeg");

            ////////////////////////////////////////////////////////final Point centerSingleMarker = new Point(getCenter(markerCornerPoints));
            ////////////////////////////////////////////////////////markerCenterPoints.add(centerSingleMarker);
            ////////////////////////////////////////////////////////Imgproc.circle(source, centerSingleMarker, 3, new Scalar(100, 100, 0), -1);
            ////////////////////////////////////////////////////////Log.i("find_ROI2D", "target2 centerSingleMarker = ("+centerSingleMarker.x + ", " + centerSingleMarker.y + ")");
        }
        ////////////////////////////////////////////////////////centerOfAllCenters = new Point(getCenter(markerCenterPoints));
        ////////////////////////////////////////////////////////Imgproc.circle(source, centerOfAllCenters, 3, new Scalar(100, 100, 0), -1);
        //////////////////////////////////////////////////////// Log.i("find_ROI2D", "centerOfAllCenters =(" + centerOfAllCenters.x + ", " + centerOfAllCenters.y + ") ");
        ////////////////////////////////////////////////////////api.saveMatImage(source, "target2 centerOfAllCenters.jpeg");

        return new Rect(minX, minY, maxX -minX, maxY-minY);
    }



    public Point undistort(Mat source, Mat cameraMatrix, Mat distCoeffs, Point point){
        float[] floatpoint = {(float) point.x, (float) point.y};
        Mat undistortedMat = new Mat(1, 2, CvType.CV_32FC2);
        undistortedMat.put(0, 0, floatpoint);
        Imgproc.undistortPoints(undistortedMat, undistortedMat, cameraMatrix, distCoeffs, new Mat(), cameraMatrix);
        Log.i("undistort", "undistortedPoint.dump()" + undistortedMat.dump());
        Point undistortedPoint = new Point(undistortedMat.get(0, 0)[0], undistortedMat.get(0, 1)[0]);
        Imgproc.circle(source, undistortedPoint, 3, new Scalar(100, 100, 0), -1);
        return undistortedPoint;
    }

    private double[] calTargetPos() {
        long start_time = SystemClock.elapsedRealtime();
        double[][] cameraParam = api.getNavCamIntrinsics();
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat dstMatrix = new Mat(1, 5, CvType.CV_32FC1);
        cameraMatrix.put(0, 0, cameraParam[0]);
        dstMatrix.put(0, 0, cameraParam[1]);

        Mat ids = new Mat();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        ArrayList<Mat> corners = new ArrayList<>();
        double xdiff = 0, ydiff = 0;

        Log.d("AR[status]:", "start");
        byte ar_count = 0;
        Rect ar_roi;
        while (ids.size().height != 4 && ar_count < 3) {
            try {
                if (ar_count == 0) {
                    ar_roi = new Rect(470, 680, 360, 280);
                }
                //fall back if the aruco tag is outside of cropped boundary.
                else {
                    ar_roi = new Rect(100, 660, 1080, 300);
                }
                Mat source = new Mat(api.getMatNavCam(), ar_roi);
                Aruco.detectMarkers(source, dictionary, corners, ids);
                for (int i = 0; i < corners.size(); i++) {
                    //Get shifted aruco tag corners
                    Mat corrected_corner = corners.get(i);
                    //Shift it by the position that it get cropped.
                    corrected_corner.put(0, 0, corrected_corner.get(0, 0)[0] + ar_roi.x, corrected_corner.get(0, 0)[1] + ar_roi.y);
                    corrected_corner.put(0, 1, corrected_corner.get(0, 1)[0] + ar_roi.x, corrected_corner.get(0, 1)[1] + ar_roi.y);
                    corrected_corner.put(0, 2, corrected_corner.get(0, 2)[0] + ar_roi.x, corrected_corner.get(0, 2)[1] + ar_roi.y);
                    corrected_corner.put(0, 3, corrected_corner.get(0, 3)[0] + ar_roi.x, corrected_corner.get(0, 3)[1] + ar_roi.y);
                    //tmp mat to store undistorted corners.
                    Mat tmp = new Mat(1, 4, CvType.CV_32FC2);
                    //undistort the corners.
                    Imgproc.undistortPoints(corners.get(i), tmp, cameraMatrix, dstMatrix, new Mat(), cameraMatrix);
                    //put it back in to the same array list.
                    corners.set(i, tmp);
                }
            } catch (Exception e) {
                Log.e("AR[status]:", "error", e);
            }
            ar_count++;
        }

        //TBV
        if (ids.size().height == 4) {
            float markerSize = 0.05f;
            double avg_ar_size = 0;
            double tx_undistort = 0, ty_undistort = 0;
            for (Mat corner : corners) {
                double _x = 0;
                double _y = 0;
                for (int j = 0; j < corner.size().width; j++) {
                    _x = _x + corner.get(0, j)[0];
                    _y = _y + corner.get(0, j)[1];
                }
                avg_ar_size += Math.abs(corner.get(0, 0)[0] - corner.get(0, 1)[0]);
                avg_ar_size += Math.abs(corner.get(0, 2)[0] - corner.get(0, 3)[0]);
                avg_ar_size += Math.abs(corner.get(0, 0)[1] - corner.get(0, 3)[1]);
                avg_ar_size += Math.abs(corner.get(0, 1)[1] - corner.get(0, 2)[1]);
                tx_undistort += _x / 4.0;
                ty_undistort += _y / 4.0;
            }
            tx_undistort /= 4;
            ty_undistort /= 4;
            avg_ar_size /= 16;
            double pixelPerM = avg_ar_size / markerSize;
            Log.d("AR[pixelperM]", "" + pixelPerM);
            //find diff from the center of the image
            xdiff = (tx_undistort - 640) / pixelPerM;
            ydiff = (480 - ty_undistort) / pixelPerM;
        }
        long stop_time = SystemClock.elapsedRealtime();
        Log.d("AR[count]", "" + ar_count);
        Log.d("AR[total_time]:", " " + (stop_time - start_time));
        Log.d("AR[target_pos]", xdiff + ", " + ydiff);
        return new double[]{xdiff, ydiff};

        //TBV
    }
    private double[] find_laser_point(double[] target_point, double r1, double r2) {
        double x1 = 0, y1 = 0, x2 = target_point[0], y2 = target_point[1];
        double centerdx = x1 - x2;
        double centerdy = y1 - y2;
        double R = length(centerdx, centerdy);
        double R2 = R * R;
        double R4 = R2 * R2;
        double a = (r1 * r1 - r2 * r2) / (2 * R2);
        double r2r2 = (r1 * r1 - r2 * r2);
        double c = Math.sqrt(2 * (r1 * r1 + r2 * r2) / R2 - (r2r2 * r2r2) / R4 - 1);
        double fx = (x1 + x2) / 2 + a * (x2 - x1);
        double gx = c * (y2 - y1) / 2;
        double ix1 = fx + gx;
        double ix2 = fx - gx;
        double fy = (y1 + y2) / 2 + a * (y2 - y1);
        double gy = c * (x1 - x2) / 2;
        double iy1 = fy + gy;
        double iy2 = fy - gy;
        if (iy1 > iy2) {
            return new double[]{ix1, iy1};
        }
        return new double[]{ix2, iy2};
    }

    public void alignLaser(Point3 target){
        double[] current_orientation = {-90, 0, 0};
        double[] laser_point = {0.1302, 0.1111};
        double pivot_laser_target_angle = 139.525787;
        double r1 = 0.1711585522;  //pivot_to_laser_length
        double[] target_point = new double[2];

        //TBV
        double[] target_pos = calTargetPos();
        target_point[0] = 0.785;
//      target_point[1] = target_pos[1] + 0.0826;
        target_point[1] = target_pos[1] + 0.0926;
        //TBV

        // law of cosines: c2=a2+b2﹣2abcosγ
        // γ = pivot_laser_target_angle
        // a = R2 (unknown)
        // b = R1
        // c= pivot_to_target_length
        Log.d("AR[target_point(xy plane)]", target_point[0] + ", " + target_point[1]);
        // square root(delta x square + delta y square)
        double pivot_to_target_length = length(target_point[0], target_point[1]);
        Log.d("AR[pivot_to_target_length]", "" + pivot_to_target_length);
        double a = 1;
        double b = 2 * r1 * Math.cos(Math.toRadians(180 - pivot_laser_target_angle));
        double c = Math.pow(r1, 2) - Math.pow(pivot_to_target_length, 2);
        double r2 = (-b + Math.sqrt(Math.pow(b, 2) - 4 * a * c)) / 2 * a;
        Log.d("AR[radius]", r1 + ", " + r2);

        double[] laser_shooting_coord = find_laser_point(target_point, r1, r2);
        Log.d("AR[laser_shooting_coord]", laser_shooting_coord[0] + ", " + laser_shooting_coord[1]);

        double laser_origin_to_shooting_length = length(laser_point[0] - laser_shooting_coord[0], laser_point[1] - laser_shooting_coord[1]);
        Log.d("AR[laser_origin_to_shooting_length]", "" + laser_origin_to_shooting_length);

        double pitch = 2 * Math.toDegrees(Math.asin((0.5 * laser_origin_to_shooting_length) / r1));
        Log.d("AR[pitch]", "" + pitch);
        if (target_point[1] != 0 && pitch < 45) {
            current_orientation[1] = current_orientation[1] - pitch;
        } else {
            //Just to be safe
            current_orientation[1] = current_orientation[1] - 30;
        }

        double[] robot_pos = {api.getRobotKinematics().getPosition().getX(), api.getRobotKinematics().getPosition().getY(), api.getRobotKinematics().getPosition().getZ()};

        //TBV
        //        robot_pos[0] = robot_pos[0] + (target_pos[0] - 0.0994);
//        robot_pos[0] = robot_pos[0] + (target_pos[0] - 0.092);
        robot_pos[0] = robot_pos[0] + (target_pos[0] - 0.1094);
        //TBV

        Log.d("AR[Robot Position]", robot_pos[0] + ", " + current_orientation[1]);
        Quaternion q = eulerToQuaternion(current_orientation[0], current_orientation[1], current_orientation[2]);
        Result result = moveToWrapper(robot_pos[0],robot_pos[1],robot_pos[2], q.getX(), q.getY(), q.getZ(), q.getW());
    }
    /*public Point correct_point(Point target_cropped) {

        byte ar_count = 0;
        while (markerIds.size().height != 4 && ar_count < 3) {
            try {
                for (int i = 0; i < markerCornerPointsID12.size(); i++) {
                    //Get shifted aruco tag corners
                    Mat corrected_corner = markerCorners.get(i);
                    //Shift it by the position that it get cropped.
                    corrected_corner.put(0, 0, corrected_corner.get(0, 0)[0] + AR_ROI.x, corrected_corner.get(0, 0)[1] + AR_ROI.y);
                    corrected_corner.put(0, 1, corrected_corner.get(0, 1)[0] + AR_ROI.x, corrected_corner.get(0, 1)[1] + AR_ROI.y);
                    corrected_corner.put(0, 2, corrected_corner.get(0, 2)[0] + AR_ROI.x, corrected_corner.get(0, 2)[1] + AR_ROI.y);
                    corrected_corner.put(0, 3, corrected_corner.get(0, 3)[0] + AR_ROI.x, corrected_corner.get(0, 3)[1] + AR_ROI.y);
                    //tmp mat to store undistorted corners.
                    Mat tmp = new Mat(1, 4, CvType.CV_32FC2);

                    //undistort the corners
                    Imgproc.undistortPoints(markerCorners.get(i), tmp, cameraMatrix, distCoeffs, new Mat(), cameraMatrix)

                    //put it back in to the same array list.
                    markerCorners.set(i, tmp);
                    Log.i("ARUCO test", "corrected_corner.dump() = "+corrected_corner.dump());
                }
            } catch (Exception e) {
                Log.e("ARUCO test:", "error", e);
            }
            ar_count++;
        }
    }*/
    public Point3 find_target_center_cam_frame(Point center, Mat rvec, Mat tvec, int id) {
        //TBV
        Mat rot_mat = new Mat();
        Calib3d.Rodrigues(rvec, rot_mat);
        Mat rmat = rot_mat.t();
        double[][] rarray = {
                new double[]{rmat.get(0, 0)[0], rmat.get(0, 1)[0], rmat.get(0, 2)[0]},
                new double[]{rmat.get(1, 0)[0], rmat.get(1, 1)[0], rmat.get(1, 2)[0]},
                new double[]{rmat.get(2, 0)[0], rmat.get(2, 1)[0], rmat.get(2, 2)[0]}
        };

        double[][] offset = {
                new double[]{center.x},
                new double[]{center.y},
                new double[]{0.0D}};

        double[][] global_offset = multiplyMat(rarray, offset);

        return( new Point3(
                global_offset[0][0] +  tvec.get(0, 0)[0],
                global_offset[1][0] +  tvec.get(1, 0)[0],
                global_offset[2][0] +  tvec.get(2, 0)[0]));
    }
}