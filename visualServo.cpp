#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#ifdef VISP_HAVE_XML2
#include <visp3/core/vpXmlParserCamera.h>
#endif
#ifdef VISP_HAVE_V4L2
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpUDPServer.h>
#include <iostream>
#include <iterator>
#include <visp3/core/vpThread.h>
#include <visp3/core/vpMutex.h>

std::string posStreammsg = "";
vpThread::Return streamFunction(vpThread::Args args);
vpMutex s_mutex_capture;
bool isCapturing = true;
bool poseUpdated = false;
bool lostTracking = false;

int main(int argc, char **argv)
{

#if defined(VISP_HAVE_APRILTAG) && (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_OPENCV))
	int opt_device = 0;
	vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
	vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
	double tagSize = 0.096;
	float quad_decimate = 1.0;
	int nThreads = 1;
	std::string intrinsic_file = "";
	std::string camera_name = "";
	bool display_tag = true;
	int color_id = -1;
	unsigned int thickness = 2;
#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
	bool display_off = true;
#else
	bool display_off = false;
#endif
	for (int i = 1; i < argc; i++) {
		if (std::string(argv[i]) == "--pose_method" && i + 1 < argc) {
			poseEstimationMethod = (vpDetectorAprilTag::vpPoseEstimationMethod)atoi(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
			tagSize = atof(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
			opt_device = atoi(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
			quad_decimate = (float)atof(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
			nThreads = atoi(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
			intrinsic_file = std::string(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
			camera_name = std::string(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--display_tag") {
			display_tag = true;
		}
		else if (std::string(argv[i]) == "--display_off") {
			display_off = true;
		}
		else if (std::string(argv[i]) == "--color" && i + 1 < argc) {
			color_id = atoi(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--thickness" && i + 1 < argc) {
			thickness = (unsigned int)atoi(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
			tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
			std::cout << "Usage: " << argv[0]
				<< " [--input <camera input>] [--tag_size <tag_size in m>]"
				" [--quad_decimate <quad_decimate>] [--nthreads <nb>]"
				" [--intrinsic <intrinsic file>] [--camera_name <camera name>]"
				" [--pose_method <method> (0: HOMOGRAPHY, 1: "
				"HOMOGRAPHY_VIRTUAL_VS,"
				" 2: DEMENTHON_VIRTUAL_VS, 3: LAGRANGE_VIRTUAL_VS,"
				" 4: BEST_RESIDUAL_VIRTUAL_VS)]"
				" [--tag_family <family> (0: TAG_36h11, 1: TAG_36h10, 2: "
				"TAG_36ARTOOLKIT,"
				" 3: TAG_25h9, 4: TAG_25h7, 5: TAG_16h5)]"
				" [--display_tag]";
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
			std::cout << " [--display_off] [--color <color id>] [--thickness <line thickness>]";
#endif
			std::cout << " [--help]" << std::endl;
			return EXIT_SUCCESS;
		}
	}
	vpCameraParameters cam;
	cam.initPersProjWithoutDistortion(794.64779, 794.67475, 635.54310, 376.88797); // Initialize camera intrinsics with previously calibrated values for the camera
#ifdef VISP_HAVE_XML2
	vpXmlParserCamera parser;
	if (!intrinsic_file.empty() && !camera_name.empty())
		parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
#endif
	std::cout << "cam:\n" << cam << std::endl;
	std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
	std::cout << "tagFamily: " << tagFamily << std::endl;
	try {
		vpImage<unsigned char> I;
		cv::VideoCapture cap(opt_device); 		// open the default camera
		cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280); // Frame could also be smaller (faster) or larger (slower)
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
		if (!cap.isOpened()) {            		// check if we succeeded in opening the camera
			std::cout << "Failed to open the camera" << std::endl;
			return EXIT_FAILURE;
		}
		cv::Mat frame;
		cv::Mat rotFrame;
		cap >> frame; 					// get a new frame from camera
		cv::rotate(frame,rotFrame,1); 	// Rotate the grabbed image frame
		vpImageConvert::convert(rotFrame, I);
		vpDisplay *d = NULL;
		if (!display_off) {
			d = new vpDisplayOpenCV(I);
		}
		vpDetectorAprilTag detector(tagFamily);
		detector.setAprilTagQuadDecimate(quad_decimate);
		detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
		detector.setAprilTagNbThreads(nThreads);
		detector.setDisplayTag(display_tag);
		std::vector<double> time_vec;
		vpThread thread_posStream((vpThread::Fn)streamFunction);	// Starting the position-streaming second thread

		vpTranslationVector transVec; 	// Translation vector
		vpRotationMatrix rotMtx; 		// Rotation matrix
		vpRxyzVector rotVec; 			// XYZ-Rotation vector
		float transGain = 250;
		float rotGain = 250.0;
		// Initial values for XYZ corrections:
		double X = 0.0;
		double Y = 0.0;
		double Z = 0.0;
		for (;;) { // Start the marker location estimation loop
			cap >> frame; 							// get a new frame from camera
			cv::rotate(frame, rotFrame, 1); 		// Rotate the grabbed image frame
			vpImageConvert::convert(rotFrame, I);	// Convert to VISP image format
			vpDisplay::display(I);					// Visualize image
			double t = vpTime::measureTimeMs();		// Benchmarking the marker detection runtime
			std::vector<vpHomogeneousMatrix> cMo_vec;
			detector.detect(I, tagSize, cam, cMo_vec); // Detect apriltags and store the detected markers in cMo_vec
			t = vpTime::measureTimeMs() - t;
			time_vec.push_back(t);
			std::stringstream ss;
			ss << "Detection time: " << t << " ms for " << detector.getNbObjects() << " tags";
			vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
			for (size_t i = 0; i < cMo_vec.size(); i++) { // For all markers detected
				vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);
				cMo_vec[i].extract(transVec);
				cMo_vec[i].extract(rotMtx);
				rotVec.buildFrom(rotMtx);
				X = (1000 * transVec[2]);
				Y = (1000 * -transVec[0]);
				Z = (1000 * -transVec[1]);
				{
					vpMutex::vpScopedLock lock(s_mutex_capture);
					posStreammsg = // Format the pose correction message (i.e. the marker location inversed) to send to the robot:
						"{X " + std::to_string(((1000 * transVec[2]) - 637) / (transGain - 50)) + // 200
						" ,Y " + std::to_string((Y + 5) / transGain) +
						" ,Z " + std::to_string((Z - 16) / transGain) +
						" ,A " + std::to_string((vpMath::deg(rotVec[1]) / rotGain)) +
						" ,B " + std::to_string((((vpMath::deg(rotVec[0])) / vpMath::abs(vpMath::deg(rotVec[0])))*(180.0 - vpMath::abs(vpMath::deg(rotVec[0])))) / rotGain) +
						" ,C " + std::to_string(-vpMath::deg(rotVec[2]) / rotGain) + " }";
					poseUpdated = true;
					lostTracking = false;
				}
				std::cout // Printing the resulting XYZABC corrections (i.e. the marker location inversed)
					<< "X " << (1000 * transVec[2]) - 637
					<< "\t Y " << Y + 5
					<< "\t Z " << Z - 16
					<< "\t A " << vpMath::deg(rotVec[1])
					<< "\t B " << (((vpMath::deg(rotVec[0])) / vpMath::abs(vpMath::deg(rotVec[0])))*(180.0 - vpMath::abs(vpMath::deg(rotVec[0]))))
					<< "\t C " << -vpMath::deg(rotVec[2]) << std::endl;
			}
			if (detector.getNbObjects() < 1)
			{
				vpMutex::vpScopedLock lock(s_mutex_capture);
				lostTracking = true;
				std::cout << "\rNo markers found...";
			}
			vpDisplay::displayText(I, 20, 20, "Click to quit.", vpColor::red);
			vpDisplay::flush(I);
			if (vpDisplay::getClick(I, false)){ // A click in the visualizer vindo terminates the application
				vpMutex::vpScopedLock lock(s_mutex_capture);
				lostTracking = true;
				isCapturing = false;
				break;
				}
	}
		std::cout << "Benchmark computation time" << std::endl;
		std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
			<< " ; " << vpMath::getMedian(time_vec) << " ms"
			<< " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;
		if (!display_off)
			delete d;
		thread_posStream.join();
}
	catch (const vpException &e) {
		std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
	}
	return EXIT_SUCCESS;
#else
	(void)argc;
	(void)argv;
#ifndef VISP_HAVE_APRILTAG
	std::cout << "ViSP is not build with Apriltag support" << std::endl;
#endif
#if !(defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_OPENCV))
	std::cout << "ViSP is not build with v4l2 or OpenCV support" << std::endl;
#endif
	std::cout << "Install missing 3rd parties, configure and build ViSP to run this tutorial" << std::endl;
#endif
	return EXIT_SUCCESS;
}

// Thread for streaming the corrected pose to an UDP stream
vpThread::Return streamFunction(vpThread::Args args)
{
	bool poseUpdatedThread = false;
	try {
		int port = 50037;
		std::string stopMsg = "{X 0.0 ,Y 0.0 ,Z 0.0 ,A 0.0 ,B 0.0 ,C 0.0 }";
		vpUDPServer server(port);
		while (true) {
			std::cout << "Listening for clients.." << std::endl;
			double start_time = vpTime::measureTimeSecond();
			std::string msg = "", hostInfo = "";
			int res = server.receive(msg, hostInfo, 5000);
			if (res) { // If a message is received on UDP
				std::cout << "Server received: " << msg << " from: " << hostInfo << std::endl;
				//Get address and port
				std::istringstream iss(hostInfo);
				std::vector<std::string> tokens;
				std::copy(std::istream_iterator<std::string>(iss),
					std::istream_iterator<std::string>(),
					std::back_inserter(tokens));
				while (true) { // Start replying the pose corrections forever
					{
						vpMutex::vpScopedLock lock(s_mutex_capture);
						if (poseUpdated) {
							poseUpdatedThread = true;
							msg = posStreammsg;
							poseUpdated = false;
						}
						if (!isCapturing) { // Send a stop message to robot on program close
							server.send(stopMsg, tokens[1], atoi(tokens[2].c_str()));
							return EXIT_SUCCESS;
						}
					}
					if (poseUpdatedThread) {
						server.send(msg, tokens[1], atoi(tokens[2].c_str()));
						poseUpdatedThread = false;
					}
					if (lostTracking){ // Send a stop message to robot if tracking is lost
						server.send(stopMsg, tokens[1], atoi(tokens[2].c_str()));
						lostTracking = false;
					}
					vpTime::wait(1); // Wait 1 milliseconds for each round
				}
			}
			else if (res == 0) {
				std::cout << "Receive timeout" << std::endl;
			}
			else {
				std::cerr << "Error server.receive()!" << std::endl;
			}
			{
				vpMutex::vpScopedLock lock(s_mutex_capture);
				if (!isCapturing)
					return EXIT_SUCCESS;
			}
		}
		return EXIT_SUCCESS;
	}
	catch (const vpException &e) {
		std::cerr << "Catch an exception: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}