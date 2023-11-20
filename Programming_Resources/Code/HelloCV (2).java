
package imageProcessing;

import android.util.Log;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Vector;
import java.lang.Math;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.ImageIcon;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.aruco.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.utils.Converters;
import org.opencv.highgui.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Date;
import java.util.List;
import java.sql.Timestamp;

public class HelloCV {
	private List<MatOfPoint3f> offset_c;

	public static Mat processedImg;
	public static Mat originalImg;
	public Mat processedCircleImg;
	public static Mat threshImg;
	public static Mat grayImg;
	public Mat sharpenImg;
	private static int kernelSize = 3;
	private static Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT,
			new Size(2 * kernelSize + 1, 2 * kernelSize + 1), new Point(kernelSize, kernelSize));
	public Mat cropped_img;
	public Mat warped_img;
	public Point text_position;
	public Rect target_rect;
	public Mat inv_rvec;
	public Mat inv_tvec;

	private static Mat ids;
	private static Mat board_ids;
	private static ArrayList<Mat> corners;
	private static List<Mat> t1_corners;
	private static Dictionary dict;
	private static Scalar borderColor;

	private static camera_params camparams;
	private static Mat new_camMatrix;
	private static float markerSize;
	private static double[] distortionArray;
	private static MatOfDouble distortion;

	///
	private static List<Mat> objP1;
	private static MatOfInt board1ID;
	private static List<Mat> objP2;
	private static MatOfInt board2ID;
	private List<Mat> objPoints1;
	private List<Mat> objPoints2;

	private static Mat rvecs;
	private static Mat tvecs;
	private static Mat rotationMatrix;

	private static Mat undistort_img;


	private static Point[] sortingPoints(MatOfPoint pts, int x, int y) {
		Point[] sortedPoints = new Point[4];
		double data[];
		for (int i = 0; i < pts.size().height; i++) {
			data = pts.get(i, 0);
			double datax = data[0];
			double datay = data[1];
//		    0-------1
//		    |		|
//		    |  x,y  |
//		    |		|
//		    2-------3
			if (datax < x && datay < y) {
				sortedPoints[0] = new Point(datax, datay);
			} else if (datax > x && datay < y) {
				sortedPoints[1] = new Point(datax, datay);
			} else if (datax < x && datay > y) {
				sortedPoints[2] = new Point(datax, datay);
			} else if (datax > x && datay > y) {
				sortedPoints[3] = new Point(datax, datay);
			}
		}
		return sortedPoints;

	}

	private static Mat sharpeningImg(Mat src) {
		Mat dst = new Mat(src.rows(), src.cols(), src.type());
		Imgproc.medianBlur(src, dst, 7);
		Core.subtract(src, dst, dst);
		Core.add(dst, src, dst);
		return dst;
	}

	public void initProcImg(Mat img) {
		processedImg = new Mat(img.rows(), img.cols(), img.type());
		originalImg = new Mat(img.rows(), img.cols(), img.type());
		img.copyTo(processedImg);
		img.copyTo(originalImg);
	}

	public void find_paper(Mat img, List<Point> src_pts) {

//		img.copyTo(processedImg);
		List<Integer> list_x = new ArrayList<Integer>();
		List<Integer> list_y = new ArrayList<Integer>();

		for (int i = 0; i < 4; i++) {
			Imgproc.circle(processedImg, src_pts.get(i), 5, new Scalar(255, 0, 0), -1);
			list_x.add((int) src_pts.get(i).x);
			list_y.add((int) src_pts.get(i).y);
		}
		Collections.sort(list_x);
		Collections.sort(list_y);

		double max_w = list_x.get(3) - list_x.get(0);
		double max_h = list_y.get(3) - list_y.get(0);
//	    1-------0
//	    |		|
//	    |  x,y  |
//	    |		|
//	    2-------3
		MatOfPoint2f dst_pts = new MatOfPoint2f(new Point(max_w - 1, 0), new Point(0, 0), new Point(0, max_h - 1),
				new Point(max_w - 1, max_h - 1));
		MatOfPoint2f _pts = new MatOfPoint2f();
		_pts.fromList(src_pts);
		cropped_ROI(originalImg, _pts);

		Mat perspective_tf = Imgproc.getPerspectiveTransform(_pts, dst_pts);
		warped_img = new Mat();
		Imgproc.warpPerspective(originalImg, warped_img, perspective_tf, new Size(max_w, max_h), Imgproc.INTER_LINEAR);
	}

	public void cropped_ROI(Mat img, MatOfPoint2f _pts) {
		target_rect = Imgproc.boundingRect(_pts);
		cropped_img = new Mat();
		cropped_img = img.submat(target_rect);
	}




	public void set_target_board1() {

		int[] id = new int[] { 1, 2, 3, 4 };
		board1ID = new MatOfInt();
		board1ID.fromArray(id);

		List<Point3> c0 = new ArrayList<>();
		List<Point3> c1 = new ArrayList<>();
		List<Point3> c2 = new ArrayList<>();
		List<Point3> c3 = new ArrayList<>();

		c0.add(new Point3(0.075f, 0.0625f, 0.0f));
		c0.add(new Point3(0.125f, 0.0625f, 0.0f));
		c0.add(new Point3(0.125f, 0.0125f, 0.0f));
		c0.add(new Point3(0.075f, 0.0125f, 0.0f));

		c1.add(new Point3(-0.125f, 0.0625f, 0.0f));
		c1.add(new Point3(-0.075f, 0.0625f, 0.0f));
		c1.add(new Point3(-0.075f, 0.0125f, 0.0f));
		c1.add(new Point3(-0.125f, 0.0125f, 0.0f));

		c2.add(new Point3(-0.125f, -0.0125f, 0.0f));
		c2.add(new Point3(-0.075f, -0.0125f, 0.0f));
		c2.add(new Point3(-0.075f, -0.0625f, 0.0f));
		c2.add(new Point3(-0.125f, -0.0625f, 0.0f));

		c3.add(new Point3(0.075f, -0.0125f, 0.0f));
		c3.add(new Point3(0.125f, -0.0125f, 0.0f));
		c3.add(new Point3(0.125f, -0.0625f, 0.0f));
		c3.add(new Point3(0.075f, -0.0625f, 0.0f));

		MatOfPoint3f c_id0 = new MatOfPoint3f();
		MatOfPoint3f c_id1 = new MatOfPoint3f();
		MatOfPoint3f c_id2 = new MatOfPoint3f();
		MatOfPoint3f c_id3 = new MatOfPoint3f();

		c_id0.fromList(c0);
		c_id1.fromList(c1);
		c_id2.fromList(c2);
		c_id3.fromList(c3);
		objPoints1 = new ArrayList<Mat>();
		objPoints1.add(c_id0);
		objPoints1.add(c_id1);
		objPoints1.add(c_id2);
		objPoints1.add(c_id3);
	}

	public void set_target_board2() {

		int[] id = new int[] { 11, 12, 13, 14 };
		board2ID = new MatOfInt();
		board2ID.fromArray(id);

		List<Point3> c0 = new ArrayList<>();
		List<Point3> c1 = new ArrayList<>();
		List<Point3> c2 = new ArrayList<>();
		List<Point3> c3 = new ArrayList<>();

		c0.add(new Point3(0.0875f, 0.0665f, 0.0f));
		c0.add(new Point3(0.1375f, 0.0665f, 0.0f));
		c0.add(new Point3(0.1375f, 0.0165f, 0.0f));
		c0.add(new Point3(0.0875f, 0.0165f, 0.0f));

		c1.add(new Point3(-0.1375f, 0.0665f, 0.0f));
		c1.add(new Point3(-0.0875f, 0.0665f, 0.0f));
		c1.add(new Point3(-0.0875f, 0.0165f, 0.0f));
		c1.add(new Point3(-0.1375f, 0.0165f, 0.0f));

		c2.add(new Point3(-0.1375f, -0.0165f, 0.0f));
		c2.add(new Point3(-0.0875f, -0.0165f, 0.0f));
		c2.add(new Point3(-0.0875f, -0.0665f, 0.0f));
		c2.add(new Point3(-0.1375f, -0.0665f, 0.0f));

		c3.add(new Point3(0.0875f, -0.0165f, 0.0f));
		c3.add(new Point3(0.1375f, -0.0165f, 0.0f));
		c3.add(new Point3(0.1375f, -0.0665f, 0.0f));
		c3.add(new Point3(0.0875f, -0.0665f, 0.0f));

		MatOfPoint3f c_id0 = new MatOfPoint3f();
		MatOfPoint3f c_id1 = new MatOfPoint3f();
		MatOfPoint3f c_id2 = new MatOfPoint3f();
		MatOfPoint3f c_id3 = new MatOfPoint3f();

		c_id0.fromList(c0);
		c_id1.fromList(c1);
		c_id2.fromList(c2);
		c_id3.fromList(c3);
		objPoints2 = new ArrayList<Mat>();
		objPoints2.add(c_id0);
		objPoints2.add(c_id1);
		objPoints2.add(c_id2);
		objPoints2.add(c_id3);
	}
	public void find_ROI3D(Mat rvec, Mat tvec) {

		List<Point3> global_corner = new ArrayList<>();

		// define paper_corner offset from its center

		Mat rot = new Mat();
		Calib3d.Rodrigues(rvec, rot);

		double[][] offset_corner = { new double[] { 0.075f, -0.075f, -0.075f, 0.075f },
				new double[] { 0.0625f, 0.0625f, -0.0625f, -0.0625f }, new double[] { 0f, 0f, 0f, 0f }, };

		double[][] rotationMatrix = { new double[] { rot.get(0, 0)[0], rot.get(0, 1)[0], rot.get(0, 2)[0] },
				new double[] { rot.get(1, 0)[0], rot.get(1, 1)[0], rot.get(1, 2)[0] },
				new double[] { rot.get(2, 0)[0], rot.get(2, 1)[0], rot.get(2, 2)[0] } };

		double[][] global_offset = multiplyMat(rotationMatrix, offset_corner);

		// c1------c0
		// | |
		// | |
		// c2------c3

		Point3 tar_c0 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][0],
				(double) tvec.get(1, 0)[0] + global_offset[1][0], (double) tvec.get(2, 0)[0] + global_offset[2][0]);

		Point3 tar_c1 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][1],
				(double) tvec.get(1, 0)[0] + global_offset[1][1], (double) tvec.get(2, 0)[0] + global_offset[2][1]);

		Point3 tar_c2 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][2],
				(double) tvec.get(1, 0)[0] + global_offset[1][2], (double) tvec.get(2, 0)[0] + global_offset[2][2]);

		Point3 tar_c3 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][3],
				(double) tvec.get(1, 0)[0] + global_offset[1][3], (double) tvec.get(2, 0)[0] + global_offset[2][3]);

		MatOfPoint3f offset_c0 = new MatOfPoint3f();
		offset_c0.fromArray(tar_c0);

		MatOfPoint3f offset_c1 = new MatOfPoint3f();
		offset_c1.fromArray(tar_c1);

		MatOfPoint3f offset_c2 = new MatOfPoint3f();
		offset_c2.fromArray(tar_c2);

		MatOfPoint3f offset_c3 = new MatOfPoint3f();
		offset_c3.fromArray(tar_c3);

		offset_c = new ArrayList<MatOfPoint3f>();
		offset_c.add(offset_c0);
		offset_c.add(offset_c1);
		offset_c.add(offset_c2);
		offset_c.add(offset_c3);

	}

	@SuppressWarnings("deprecation")
	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		System.out.println("load success");

		double fx = 567.22931;
		double cx = 659.07721;
		double fy = 574.19293;
		double cy = 517.00757;

		double[] camArray = new double[] { fx, 0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 };
		distortionArray = new double[] { -0.21624701, 0.03875, -0.010157, 0.0019690001, 0 };

		Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
		cameraMatrix.put(0, 0, camArray);// (row, col,int[])

		Mat distCoeffs = new Mat(1, 5, CvType.CV_64FC1);
		dstMatrix.put(0, 0, distortionArray);

		distortion = new MatOfDouble();
		distortion.fromArray(distortionArray);

		set_target_board1();
		set_target_board2();
		objP1 = objPoints1;
		objP2 = objPoints2;

		String file_path = "C:\\Users\\Elyes\\Desktop\\Kibo-2022\\INPUT\\target2.jpeg";
//		String file_path = "\\your\\image\\path\\filename.png";
		Mat read_img = Imgcodecs.imread(file_path);

		//Mat read_img = api.getMatNavCam();
		//api.saveMatImage(read_img, "test_board read_img.jpeg");
		//Log.i("test_board", "read_img.size() = "+ read_img.size());

		initProcImg(read_img);

		corners = new ArrayList<>();
		ids = new Mat();

		dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
		board = Board.create(objP2, dict, board2ID);
		Aruco.detectMarkers(processedImg, dict, corners, ids);

		Mat b_rvec = new Mat();
		Mat b_tvec = new Mat();

		Aruco.estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, b_rvec, b_tvec);
		System.out.println("test board b_rvec.dump()" + b_rvec.dump());
		System.out.println("test board b_tvec.dump()" + b_tvec.dump());

		//Log.i("test_board ", "b_rvec.dump() = " + b_rvec.dump());
		//Log.i("test_board ", "b_tvec.dump() = " + b_tvec.dump());

		find_ROI3D(b_rvec, b_tvec);
		List<MatOfPoint3f> offset =	offset_c;

		double tarx = (double) b_tvec.get(0, 0)[0];
		double tary = (double) b_tvec.get(1, 0)[0];
		double tarz = (double) b_tvec.get(2, 0)[0];

		Point3 tar_pos = new Point3(tarx, tary, tarz);

		MatOfPoint3f _target3D = new MatOfPoint3f();
		_target3D.fromArray(tar_pos);
		MatOfPoint2f _targetImagePlane = new MatOfPoint2f();
		Mat _rvec = new Mat(1, 3, CvType.CV_64FC1);
		Mat _tvec = new Mat(1, 3, CvType.CV_64FC1);

		double[] _r = new double[] { 0.0f, 0.0f, 0.0f };
		double[] _t = new double[] { 0.0f, 0.0f, 0.0f };
		_rvec.put(0, 0, _r);
		_tvec.put(0, 0, _t);

		// find center of marker in 2D image
		Calib3d.projectPoints(_target3D, _rvec, _tvec, cameraMatrix, distortion, _targetImagePlane);

		int cpx = (int) _targetImagePlane.get(0, 0)[0];
		int cpy = (int) _targetImagePlane.get(0, 0)[1];
		Point center = new Point(cpx, cpy);
//		Imgproc.circle(processedImg, center, 4, new Scalar(0,255,255), -1);
		Calib3d.drawFrameAxes(processedImg, camMatrix, distCoeffs, b_rvec, b_rvec, (float) 0.05);

		List<Point> ROI_points = new ArrayList<Point>();

		for (int i = 0; i < 4; i++) {

			Calib3d.projectPoints(offset.get(i), _rvec, _tvec, cameraMatrix, distortion, _targetImagePlane);

			// without distortion parameter
//			Calib3d.projectPoints(offset.get(i), _rvec,_tvec,cameraMatrix, new MatOfDouble(), _targetImagePlane);

			int _cpx = (int) _targetImagePlane.get(0, 0)[0];
			int _cpy = (int) _targetImagePlane.get(0, 0)[1];
			Point _center = new Point(_cpx, _cpy);
			System.out.print(offset.get(i).get(0, 0)[0] + " " + offset.get(i).get(0, 0)[1] + " "
					+ offset.get(i).get(0, 0)[2] + " ");
			System.out.println(_center);
			//Log.i ("test_baord" ,offset.get(i).get(0, 0)[0] + " " + offset.get(i).get(0, 0)[1] + " " + offset.get(i).get(0, 0)[2] + " ");
			//Log.i ("test_board", "center= "+ _center.y + ", " + _center.y);
			ROI_points.add(_center);
		}
		// find Region of interest
		find_paper(processedImg, ROI_points);
		// now we got warped_img , and cropped_img

		// find contour
//		System.out.println((int)(cropped_img.width()/2));
//		Imgproc.circle(cropped_img, new Point((int)(.cropped_img.width()/2),(int)(cropped_img.height()/2)), 5, new Scalar(0,0,255), -1);

		Imgproc.rectangle(processedImg, target_rect, new Scalar(0, 0, 255));

		// set cropped image back to original image
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchey = new Mat();
		Mat cropped_gray = new Mat();

		Imgproc.cvtColor(cropped_img, cropped_gray, Imgproc.COLOR_BGR2GRAY);
		Mat binaryImg = new Mat();
		Imgproc.threshold(cropped_gray, binaryImg, 100, 200, Imgproc.THRESH_BINARY_INV);

		Imgproc.findContours(binaryImg, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
//		System.out.println(contours.size());
		for (int i = 0; i < contours.size(); i++) {
			Scalar color = new Scalar(0, 255.0, 0);
			// Drawing Contours
			if (hierarchey.get(0, i)[2] == -1.0) {
				MatOfPoint2f ct2f = new MatOfPoint2f(contours.get(i).toArray());
				Moments moment = Imgproc.moments(ct2f);

				int x = (int) (moment.get_m10() / moment.get_m00());
				int y = (int) (moment.get_m01() / moment.get_m00());

				Imgproc.circle(cropped_img, new Point(x, y), 2, new Scalar(0, 0, 255), -1);
			}
			Imgproc.drawContours(cropped_img, contours, i, color, 1, Imgproc.LINE_8, hierarchey, 1,
					new Point());
		}

		cropped_img.copyTo(processedImg.submat(target_rect));

		//api.saveMatImage(processedImg, "test_board.jpeg");
		//api.saveMatImage(warped_img, "warped.jpeg", );
		//api.saveMatImage(cropped_img, "cropped.jpeg", );
		//api.saveMatImage(binaryImgn, "binary.jpeg", );

		HighGui.imshow("output", processedImg);
		HighGui.imshow("warped", warped_img);
		HighGui.imshow("cropped",cropped_img);
		HighGui.imshow("binary", binaryImg);

		String output_file = "C:\\Users\\Elyes\\Desktop\\Kibo-2022\\OUTPUT\\output.jpeg";
		Imgcodecs.imwrite(output_file, processedImg);
		HighGui.waitKey();
	}

}
