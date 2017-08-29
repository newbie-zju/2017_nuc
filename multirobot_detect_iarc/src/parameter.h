/*
#define IrobotSetNo 218			  
#define ObstacleSetNo 205		
#define BackgroundSetNo 736	
#define HardBackgroundSetNo 155

#define SHOWSET false
#define TRAIN true
#define SAVESET false
*/
/*//for cpu
//HOG descriptor
#define WinSizeDetect Size(32,16)
#define BlockSizeDetect Size(8,8)
#define BlockStrideDetect Size(4,4)
#define CellSizeDetect Size(4,4)
#define NbinsDetect 9	

//HOG descriptor
#define WinSizeClassify Size(48,24)
#define BlockSizeClassify Size(16,8)
#define BlockStrideClassify Size(4,4)
#define CellSizeClassify Size(4,4)
#define NbinsClassify 9	
*/

///////////for GPU
//HOG descriptor
#define WinSizeDetect Size(40,40)
#define BlockSizeDetect Size(16,16)
#define BlockStrideDetect Size(8,8)
#define CellSizeDetect Size(8,8)
#define NbinsDetect 9

//HOG descriptor
#define WinSizeClassify Size(40,40)
#define BlockSizeClassify Size(16,16)
#define BlockStrideClassify Size(8,8)
#define CellSizeClassify Size(8,8)
#define NbinsClassify 9	

//HOG descriptor
//#define WinSize Size(20,10)
//#define BlockSize Size(4,4)
//#define BlockStride Size(2,2)
//#define CellSize Size(2,2)
//#define Nbins 9

//HOG descriptor
//#define WinSize Size(64,32)
//#define BlockSize Size(8,8)
//#define BlockStride Size(4,4)
//#define CellSize Size(4,4)
//#define Nbins 9

//partial parameters of detectMultiScale
#define HitThreshold 0		//bias of hyperplane
#define WinStride Size(8,8)	//it must be the integer times of the block stride in HOG descriptor
#define DetScale 1.1		//reduced proportion of image every time
#define SuppressionRate 0.4     //value for non maximum suppression
#define BBOverlapRate 0.2	//BBOverlapRate for filter
#define detect_resize_rate 1.4	//boxes' resizerate of detect's output


//#define TestImage "../Data/TestImage/13.jpg"
//#define ResultImage "../Data/Result/13.jpg"
//#define ResultImageFile_1 "..\\Data\\Result\\13-1\\"
//#define ResultImageFile_2 "..\\Data\\Result\\13-2\\"
//#define ResultImageFile_3 "..\\Data\\Result\\13-3\\"
//#define TestVideo "/home/ubuntu/ros_my_workspace/src/multirobot_detect/data/test_video/a544.avi"
//#define ResultVideo "/home/ubuntu/ros_my_workspace/src/multirobot_detect/data/result/a544.avi"
//#define ResultVideoFile_1 "..\\Data\\Result\\1-1\\"
//#define ResultVideoFile_2 "..\\Data\\Result\\1-2\\"
//#define ResultVideoFile_3 "..\\Data\\Result\\1-3\\"

//#define IrobotSetFile "../Data/IrobotSet/"
//#define ObstacleSetFile "../Data/ObstacleSet/"
//#define BackgroundSetFile "../Data/BackgroundSet/"
//#define HardBackgroundSetFile "../Data/HardBackgroundSet/"
//#define SetName "0SetName.txt"
#define DetectSvmName "/home/zmart/2017/src/multirobot_detect_iarc/src/SVM_HOG_Detect.xml"
#define ClassifySvmName "/home/zmart/2017/src/multirobot_detect_iarc/src/SVM_HOG_Classify.xml"

/*
#define TrainPerc 0.8	//proportion of train set
#define VaildPerc 0.2	//proportion of cross vaild set
//#define TestPerc 0.2
#define IrobotTrainNo (int(IrobotSetNo * TrainPerc))//number of train set
#define ObstacleTrainNo (int(ObstacleSetNo * TrainPerc))
#define BackgroundTrainNo (int(BackgroundSetNo * TrainPerc))
#define HardBackgroundTrainNo (int(HardBackgroundSetNo * TrainPerc))
#define AllTrainNo (IrobotTrainNo + ObstacleTrainNo + BackgroundTrainNo + HardBackgroundTrainNo)
#define IrobotVaildNo (int(IrobotSetNo * VaildPerc))//number of cross vaild set
#define ObstacleVaildNo (int(ObstacleSetNo * VaildPerc))
#define BackgroundVaildNo (int(BackgroundSetNo * VaildPerc))
#define HardBackgroundVaildNo (int(HardBackgroundSetNo * VaildPerc))
#define AllVaildNo (IrobotVaildNo + ObstacleVaildNo + BackgroundVaildNo + HardBackgroundVaildNo)
#define IrobotTestNo (IrobotSetNo - IrobotTrainNo - IrobotVaildNo)//number of test set
#define ObstacleTestNo (ObstacleSetNo - ObstacleTrainNo - ObstacleVaildNo)
#define BackgroundTestNo (BackgroundSetNo - BackgroundTrainNo - BackgroundVaildNo)
#define HardBackgroundTestNo (HardBackgroundSetNo - HardBackgroundTrainNo - HardBackgroundVaildNo)
#define AllTestNo (IrobotSetNo + ObstacleSetNo + BackgroundSetNo + HardBackgroundSetNo)
*/
