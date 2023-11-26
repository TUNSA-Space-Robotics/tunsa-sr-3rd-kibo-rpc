
package imageProcessing;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.util.ArrayList;
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

	private static int dictID = Aruco.DICT_5X5_250;
	private static Mat ids;
	private static Mat board_ids;
	private static ArrayList<Mat> corners;
	private static List<Mat> t1_corners;
	private static Dictionary dict;
	private static Scalar borderColor;

	private static camera_params camparams;
	private static Mat camMatrix;
	private static Mat new_camMatrix;
	private static Mat dstMatrix;
	private static float markerSize;
	private static double[] distortionArray;
	private static MatOfDouble distortion;

	///
	private static List<Mat> objP1;
	private static MatOfInt board1ID;
	private static List<Mat> objP2;
	private static MatOfInt board2ID;
	static target_board t_board1 = new target_board();
	static target_board t_board2 = new target_board();
	///

	private static Mat rvecs;
	private static Mat tvecs;
	private static Mat t1_rvec;
	private static Mat t1_tvec;
	private static Mat rotationMatrix;

	private static Mat undistort_img;

	private static imgProcessing imgProc;

	public static void setCamCalibration() {
		camparams = new camera_params();
		camparams.process_camera();

		camMatrix = camparams.getCamMatrix();
		dstMatrix = camparams.getDistortionMat();
		distortionArray = camparams.getDistortionArray();
		distortion = new MatOfDouble();
		distortion.fromArray(distortionArray);
	}

	public static void setBoard() {

		t_board1.set_target_board1();
		t_board2.set_target_board2();
		objP1 = t_board1.getObjP1();
		board1ID = t_board1.getBoard1ID();

		objP2 = t_board2.getObjP2();
		board2ID = t_board2.getBoard2ID();

	}

	@SuppressWarnings("deprecation")
	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		System.out.println("load success");

		setCamCalibration();
		setBoard();

		String file_path = "C:\\Users\\jiraphan\\Desktop\\KIBO-2022\\navcamIMGs\\target2_1.png";
//		String file_path = "\\your\\image\\path\\filename.png";

		Mat read_img = Imgcodecs.imread(file_path);
		System.out.println(read_img.size());

		imgProc = new imgProcessing();

		imgProc.initProcImg(read_img);

		corners = new ArrayList<>();
		ids = new Mat();

		dict = Aruco.getPredefinedDictionary(dictID);
		////////////////////////////////////////////////////
		Board t1_board = Board.create(objP1, dict, board1ID);
		Board t2_board = Board.create(objP2, dict, board2ID);
		////////////////////////////////////////////////////
		Aruco.detectMarkers(imgProc.processedImg, dict, corners, ids);

		t1_rvec = new Mat();
		t1_tvec = new Mat();
		Aruco.estimatePoseBoard(corners, ids, t2_board, camMatrix, dstMatrix, t1_rvec, t1_tvec);

		t_board2.find_ROI3D(t1_rvec, t1_tvec);
		List<MatOfPoint3f> offset = t_board2.get_offset();

		double tarx = (double) t1_tvec.get(0, 0)[0];
		double tary = (double) t1_tvec.get(1, 0)[0];
		double tarz = (double) t1_tvec.get(2, 0)[0];

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
		Calib3d.projectPoints(_target3D, _rvec, _tvec, camMatrix, distortion, _targetImagePlane);

		int cpx = (int) _targetImagePlane.get(0, 0)[0];
		int cpy = (int) _targetImagePlane.get(0, 0)[1];
		Point center = new Point(cpx, cpy);
//		Imgproc.circle(imgProc.processedImg, center, 4, new Scalar(0,255,255), -1);
		Calib3d.drawFrameAxes(imgProc.processedImg, camMatrix, dstMatrix, t1_rvec, t1_rvec, (float) 0.05);
		//

		List<Point> ROI_points = new ArrayList<Point>();

		for (int i = 0; i < 4; i++) {

			Calib3d.projectPoints(offset.get(i), _rvec, _tvec, camMatrix, distortion, _targetImagePlane);

			// without distortion parameter
//			Calib3d.projectPoints(offset.get(i), _rvec,_tvec,camMatrix, new MatOfDouble(), _targetImagePlane);

			int _cpx = (int) _targetImagePlane.get(0, 0)[0];
			int _cpy = (int) _targetImagePlane.get(0, 0)[1];
			Point _center = new Point(_cpx, _cpy);
			System.out.print(offset.get(i).get(0, 0)[0] + " " + offset.get(i).get(0, 0)[1] + " "
					+ offset.get(i).get(0, 0)[2] + " ");
			System.out.println(_center);
			ROI_points.add(_center);
		}
		// find Region of interest
		imgProc.find_paper(imgProc.processedImg, ROI_points);
		// now we got warped_img , and cropped_img

		// find contour
//		System.out.println((int)(imgProc.cropped_img.width()/2));
//		Imgproc.circle(imgProc.cropped_img, new Point((int)(imgProc.cropped_img.width()/2),(int)(imgProc.cropped_img.height()/2)), 5, new Scalar(0,0,255), -1);

		Imgproc.rectangle(imgProc.processedImg, imgProc.target_rect, new Scalar(0, 0, 255));

		// set cropped image back to original image
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchey = new Mat();
		Mat cropped_gray = new Mat();

		Imgproc.cvtColor(imgProc.cropped_img, cropped_gray, Imgproc.COLOR_BGR2GRAY);
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

				Imgproc.circle(imgProc.cropped_img, new Point(x, y), 2, new Scalar(0, 0, 255), -1);
//	     		
			}
			Imgproc.drawContours(imgProc.cropped_img, contours, i, color, 1, Imgproc.LINE_8, hierarchey, 1,
					new Point());
		}

		imgProc.cropped_img.copyTo(imgProc.processedImg.submat(imgProc.target_rect));

		HighGui.imshow("output", imgProc.processedImg);
		HighGui.imshow("warped", imgProc.warped_img);
		HighGui.imshow("cropped", imgProc.cropped_img);
		HighGui.imshow("binary", binaryImg);

//		String output_file = "\\your\\output\\path\\filename.png";
//		Imgcodecs.imwrite(output_file, imgProc.processedImg);
		HighGui.waitKey();
	}

}
