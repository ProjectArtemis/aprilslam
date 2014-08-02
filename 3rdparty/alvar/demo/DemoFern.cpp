/**************************************************************************************
* Experimental
*
* Example how to use ALVAR's markerless tracking (Fern classifier) with osgViewer.
* 
* This sample detects image marker (predefined, or command-line parameter) from the view. 
* When the field is detected, system adds the model on top of it. 
* Before the image can be used for tracking, the program must enter a training phase,
* where the Fern classifier is trained for the selected image. For a 200x200 image
* this takes about one minute.
* 
* Model is a basic OSG model (download the models from the OpenSceneGraph's webpage)
**************************************************************************************/



// OSG Includes
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osg/MatrixTransform>
#include <osg/Depth>
#include <osg/BlendFunc>
#include <osg/BlendColor>
#include <osg/PositionAttitudeTransform>
#include <osgUtil/Optimizer>
#include <osg/Material>
#include <osg/CullFace>
#include <osg/PolygonMode>
#include <osg/LineWidth>
#include <osg/Point>

// ALVAR Includes
#include <Camera.h>
#include <Pose.h>
#include <MarkerDetector.h>
#include <CaptureFactory.h>
#include <FernImageDetector.h>
#include <FernPoseEstimator.h>

// Own includes
#include "Shared.h"



osg::ref_ptr<osgViewer::Viewer> viewer;
osg::ref_ptr<osg::Node> model_node;
osg::ref_ptr<osg::Switch> model_switch;
osg::ref_ptr<osg::Group> root_group;
osg::ref_ptr<osg::MatrixTransform> model_transform;
ViewWithBackGroundImage *viewBG;
alvar::FernImageDetector fernDetector(true);
alvar::FernPoseEstimator fernEstimator;

alvar::Capture *capture = 0;
IplImage *frame = 0;
IplImage *gray  = 0;
IplImage *rgb   = 0;
int _width=0, _height=0;
int _origin=0;

bool stop_running  = false;



/*
	The keyboard handler.
*/
class PickHandler : public osgGA::GUIEventHandler 
{
public:

	PickHandler() {}
	~PickHandler() {}

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
        if (!view) 
			return false;

        switch(ea.getEventType())
        {
			case(osgGA::GUIEventAdapter::KEYDOWN):
			{
				switch(ea.getKey())
				{
					case 'q':
						{
							stop_running = true;
							std::cout << "Stopping..." << std::endl;
						} 
						break;
				}
			}
		}			
        return true;
	}
};



/*
	Initialize video capture.
*/
bool InitVideoCapture(size_t use_cap) 
{
    // Enumerate possible capture devices and check the command line argument which one to use
	std::cout<<"Enumerated Capture Devices:"<<std::endl;
	alvar::CaptureFactory::CaptureDeviceVector vec = alvar::CaptureFactory::instance()->enumerateDevices();

	if (vec.size() < 1) 
	{
		std::cout<<"  none found"<<std::endl;
		return false;
	}
	if (use_cap >= vec.size()) 
	{
		std::cout<<"  required index not found - using 0"<<std::endl;
		use_cap=0;
	}
	for (size_t i=0; i<vec.size(); i++) 
	{
        if (use_cap == i) 
			std::cout << "* "; 
		else 
			std::cout << "  ";
		std::cout << i << ": " << vec[i].uniqueName();
        if (vec[i].description().length() > 0) 
			std::cout << ", " << vec[i].description();
		std::cout << std::endl;
	}
    std::cout << std::endl;
	capture = alvar::CaptureFactory::instance()->createCapture(vec[use_cap]);

	if (capture) 
	{
		std::stringstream filename;
		filename << "camera_settings_" << vec[use_cap].uniqueName() << ".xml";
		capture->start();
		if (capture->loadSettings(filename.str())) 
			std::cout << "read: " << filename.str() << std::endl;

		int i;
		IplImage *dummy;
		for (i=0; i<10; i++) {
			dummy = capture->captureImage();
			if (dummy) 
				break;
// todo: why is a sleep necessary?
            alvar::sleep(100);
		}
		if (i == 10) 
			return false;
		_width  = dummy->width;
		_height = dummy->height;
		_origin = dummy->origin;
		return true;
	}
	return false;
}



/*
	Allocate memory for one RGB image and one grayscale image.
*/
bool InitImages(int w, int h, int origin)
{
	if (w==0||h==0) 
		return false;
	CvSize size = cvSize(w, h);
	if (!rgb)
	{
		rgb  = cvCreateImage(size, 8, 3);
		rgb->origin = origin;
	}
	if (!gray)
	{
		gray = cvCreateImage(size, 8, 1);
		gray->origin = origin;
	}
	return true;
}



/*
	Initialize OSG, create a Viewer
*/
bool InitOSG(int w, int h, bool flip_image, std::string model_file, std::string camera_file)
{
	int offx = 10;
	int offy = 30;

	viewer = new osgViewer::Viewer;
	osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
    if (!wsi)
    {
        osg::notify(osg::NOTICE) << "Error, no WindowSystemInterface available, cannot create windows." << std::endl;
        return false;
    }

    unsigned int width, height;
    wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), width, height);

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x	   = offx;
    traits->y	   = offy;
    traits->width  = w;
    traits->height = h;
    traits->windowDecoration = true;
    traits->doubleBuffer	 = true;
    traits->sharedContext	 = 0;

    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
    if (gc.valid())
    {
        osg::notify(osg::INFO) << "  GraphicsWindow has been created successfully." << std::endl;
        // need to ensure that the window is cleared make sure that the complete window is set the correct colour
        // rather than just the parts of the window that are under the camera's viewports
        gc->setClearColor(osg::Vec4f(0.2f,0.2f,0.6f,1.0f));
        gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    else
    {
        osg::notify(osg::NOTICE) << "  GraphicsWindow has not been created successfully." << std::endl;
    }

	model_node = osgDB::readNodeFile(model_file.c_str());
	model_transform = new osg::MatrixTransform;
	model_transform->addChild(model_node.get());
	model_switch = new osg::Switch;
	model_switch->addChild(model_transform.get());
	model_switch->setAllChildrenOff();

	root_group = new osg::Group;
	root_group->addChild(model_switch.get());

	osg::StateSet *state_set = new osg::StateSet;
	state_set->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	model_node->setStateSet(state_set);
	root_group->setStateSet(state_set);
	
    viewer->getCamera()->setViewport(new osg::Viewport(0, 0, w, h));
    viewer->getCamera()->setGraphicsContext(gc.get());
	viewer->addEventHandler(new PickHandler());
	viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

	viewBG = new ViewWithBackGroundImage(viewer, w, h, flip_image, root_group);

	alvar::Camera cam;
	cam.SetCalib(camera_file.c_str(), w, h);
	double p[16];
	cam.GetOpenglProjectionMatrix(p, w, h);
	viewer->getCamera()->setProjectionMatrix(osg::Matrix(p));

	return true;
}



/*
	Initialize Fern classifier
*/
bool InitFernClassifier(int w, int h, std::string calib_file, std::string image_marker_file)
{
	std::string classifierFilename = image_marker_file + ".dat";

	std::cout << "Trying to load classifier... ";
	if (fernDetector.read(classifierFilename))
	{
		std::cout << "Existing classifier found and loaded." << std::endl;
		return true;
	}
	std::cout << "No existing classifier found." << std::endl;

	std::cout << "Training Fern classifier..." << std::endl;
	fernDetector.train(image_marker_file);
    std::cout << "Fern classifier Trained." << std::endl;
	fernEstimator.setCalibration(calib_file, w, h);

	std::cout << "Writing classifier." << std::endl;
	if (!fernDetector.write(classifierFilename))
		std::cout << "Writing classifier failed." << std::endl;
	return true;
}



/*
	Clean-up: free memory allocated for the RGB and grayscale images, stop capture.
*/
void CleanUp()
{
	if (gray) 
		cvReleaseImage(&gray);
	if (rgb)  
		cvReleaseImage(&rgb);
	if (capture) 
	{			
		//capture->saveSettings(filename.str());
		capture->stop();
		delete capture;
		capture = 0;
	}
}



/*
	Process each captured video frame.
*/
void Process()
{
	while (true)
	{
		frame = capture->captureImage();
		if (frame == 0) 
			continue; // TODO: For some reason CvCam gives NULL sometimes?
		if(frame->origin==1) 
		{ 
			cvFlip(frame); 
			frame->origin=0;
		}
		if (frame->nChannels == 1) 
		{
			gray->imageData = frame->imageData;
			cvCvtColor(frame, rgb, CV_BayerGB2RGB); // TODO: Now this assumes Bayer
		} 
		else 
		{
			rgb->imageData = frame->imageData;
			cvCvtColor(frame, gray, CV_RGB2GRAY);
		}
		gray->origin = frame->origin;

		bool track_ok = false;
		vector<CvPoint2D64f> ipts;
		vector<CvPoint3D64f> mpts;
		cv::Mat grayMat = cv::Mat(gray);
		fernDetector.findFeatures(grayMat, true);
		fernDetector.imagePoints(ipts);
		fernDetector.modelPoints(mpts);
		double test = fernDetector.inlierRatio();
		if (test>0.25 && mpts.size()>12)
		{
			fernEstimator.calculateFromPointCorrespondences(mpts, ipts);
			track_ok = true;
//			std::cout << "\rTrack OK: " << test << ", " << mpts.size() << "   ";
		}

		if (track_ok)
		{
			// Draw cameras, points & features
			alvar::Pose p = fernEstimator.pose();
			double gl_mat[16];
			p.GetMatrixGL(gl_mat);
			model_transform->setMatrix(osg::Matrix(gl_mat));
			// The pose Z seems to be into the paper, while for normal markers it is out of the paper;
			// let's flip the pose so the XYZ osg model looks the same...
			model_transform->preMult(osg::Matrix::rotate(osg::PI, osg::Vec3(1,0,0)));
			model_switch->setAllChildrenOn();
		}
		else
		{
			model_switch->setAllChildrenOff();
//			std::cout << "\rTrack Failed     ";
		}

		viewBG->DrawImage(rgb);
		if (viewer->done())
			break;
		viewer->frame();
		if (stop_running) 
			break;
	}
}



int main(int argc, char **argv)
{	
    // Output usage message
    std::cout << "OsgFerns example" << std::endl;
	std::cout << "==============" << std::endl;
    std::cout << std::endl;
    std::cout << "Description:" << std::endl;
    std::cout << "  This is an example of how to use the 'FernImageDetector' class " << std::endl;
	std::cout << "  to train a Fern classifier for markerless image-based tracking." << std::endl;
    std::cout << "  The file name of the marker image is entered as the command-line parameter." << std::endl;
	std::cout << "  The size of the image should be about 200x200 pixels," << std::endl;
	std::cout << "  and should contain many unique features." << std::endl;
    std::cout << "  First the program enters the Fern training phase (takes about one minute)," << std::endl;
    std::cout << "  after that the image is used as a marker," << std::endl;
    std::cout << "  and an OSG model is augmented on top of it." << std::endl;
    std::cout << std::endl;
    std::cout << "Possible command line parameters:" << std::endl;
    std::cout << "  <the number of the capture device to use>" << std::endl;
    std::cout << "  <camera calibration file>" <<  std::endl;
    std::cout << "  <image file>" << std::endl;
    std::cout << std::endl;
	std::cout << "Keyboard Shortcuts (these work only when AR window is selected):" << std::endl;
    std::cout << "  q: quit" << std::endl;
    std::cout << std::endl;

	size_t use_cap = 0;
	std::string calib_file = "calib.xml";
	std::string img_mkr_file = "markerless.png";
	if (argc > 1) 
		use_cap = atoi(argv[1]);
	if (argc > 2) 
		calib_file = argv[2];
	if (argc > 3) 
		img_mkr_file = argv[3];

	if (InitVideoCapture(use_cap) && 
		InitImages(_width, _height, _origin) && 
		InitOSG(_width, _height, true, "axes.osg", calib_file) &&
		InitFernClassifier(_width, _height, calib_file, img_mkr_file)) 
	{
		Process();
		CleanUp();
		return 0;
	}
	CleanUp();
	return -1;
}
