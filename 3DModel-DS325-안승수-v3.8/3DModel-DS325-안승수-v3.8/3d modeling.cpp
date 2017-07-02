
//DepthMap_DS325.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.

#include "stdafx.h"

/////////////////////////////////////////////DS325 Code
#ifdef _MSC_VER
#include <windows.h>
#endif

#include <vector>
#include <exception>

#include <DepthSense.hxx>
using namespace DepthSense;
using namespace std;

Context g_Context;
DepthNode g_DNode;

int g_uWidthDS;
int g_uHeightDS;

BOOL g_bIsCloseMode=0;
BOOL g_bDenoising=1;
int g_uLight=100;
int g_uFrameRate=60;
int g_uConfidence=100;

IplImage* g_imgDepthBuffer;
BOOL g_bFlagNewFrame=false;
CRITICAL_SECTION g_csDepthFrame;
HANDLE g_hThreadDepth;
DWORD g_IDThreadDepth;
volatile bool g_bClose=false;

ColorNode g_CNode;
int g_uColorWidthDS;
int g_uColorHeightDS;
IplImage* g_imgColorBuffer;
BOOL g_bFlagNewColorFrame=false;
CRITICAL_SECTION g_csColorFrame;
CRITICAL_SECTION g_csCalibrationFrame;
IplImage *g_imgY;
IplImage *g_imgCb;
IplImage *g_imgCr;
IplImage* g_imgCalBuffer;
IplImage* g_imgCalibration;
//////////////////////////////////DS325 Code ---

#define GET2D8U(IMAGE,X,Y)	(*( ( (uchar*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + (X) ))
#define GET2D8U3CH(IMAGE,X,Y) ( ( (uchar*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + ( 3 * (X) ) )
//use :  GET2D8U3CH(IMAGE,X,Y)[0], GET2D8U3CH(IMAGE,X,Y)[1], GET2D8U3CH(IMAGE,X,Y)[2]
#define GET2D16U(IMAGE,X,Y)	(*( ( (ushort*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + (X) ))
#define GET2D16S(IMAGE,X,Y)	(*( ( (short*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + (X) ))
#define GET2D32F(IMAGE,X,Y) (*( ( (float*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + (X) ))

#define DEPTH_WIDTH 320
#define DEPTH_HEIGHT 240

#define COLOR_WIDTH 320
#define COLOR_HEIGHT 240
////////////////////////////////////////////////////from  좌표 뽑기

bool onoff = false;

typedef struct _positionDot
{
	float x, y, z, b, g, r, m_z;
	/// 유효한 좌표 선택
	bool g_tv; //g_truevertex

} positionDot;

//positionDot p[3000000];
positionDot *p,*p2;

int cnt = 0;
////////////////////////////////


//CV_MAT_ELEM_PTR
CvMemStorage* g_memStorage;
int g_uRangeMin;
int g_uRangeMax;

int g_uMaxDepth;
int g_uMinDepth;

int g_uThreshold_min=0;
int g_Difference; // 18
int g_colorenalbe=1;
int g_holefilling=1;
int g_optitrack=0;
int g_xmove; //89
int g_ymove; //50
int g_xresize;//119
int g_yresize;//98
int g_uThreshold_max=255;

IplImage* g_imgColor;
IplImage* g_imgColorcal;
IplImage* g_colorcalspace;
//IplImage* g_imgDepthRaw;
IplImage* g_imgDepth;
IplImage* g_imgDepthRangedGray;
IplImage* g_imgBinary;
IplImage* g_bilateral;
IplImage* g_depthresize;
IplImage* g_display;
IplImage* g_display_left;
IplImage* g_display_right;
IplImage* g_displayzoomin;
IplImage* g_displayzoomleft;
IplImage* g_displayzoomright;
IplImage* g_display_stereo;
IplImage* g_tempdepth;

char str_fps[20]="0.00fps";

static float theta;
static int direction;
static int rotate_x;
static int rotate_y;
static int rotate_z;
bool rt;

int cnt_points, depth_max, depth_min;

int windowleft;
int windowright;

/// 추가////
//int addc=1;
//int addc2=1;
int g_dtotal;
int g_dtotal2;

static int pre_X = 0;
static int pre_Y = 0;
static int pre_Z = 0;
//좌표보정 수치 조정 (default 0.5)
const float ratio1 = 0.5;
static BOOL bInit1 = FALSE;

/////////////////최근접 깊이 값 찾기 위해
int g_temp=0;
int g_temp2=0;
int g_dvmax=0;
int g_dvmax2=0;
int g_dvmin=255;
int g_dvmid=0;
int g_dvmid2=0;
int g_dvmids=0;
int g_maxX=0;
int g_maxY=0;
int g_minX=640;
int g_minY=480;
/// 유효한 좌표 선택
bool g_tv=true; //g_TrueVertex
bool check =false;

//////////////////////////////////////
//마우스로 축회전
GLint  mouseX = 0;
GLint  mouseY = 0;
GLint  mouseState = 0;
GLint  mouseButton = 0;
int  g=1;
GLfloat  xTheta=0.0, yTheta=0.0;
GLfloat  scale=1.0, scaleDelta=1.01; 

///////////////////////////////////////
//winapi window 화면 띄우기 선언
LRESULT CALLBACK WndProc(HWND hWnd, UINT iMessage, WPARAM wParam, LPARAM IParam);
DWORD __stdcall CreateStereoWindowsAPI(void* lpszClass);
HANDLE g_hThreadTvDisplay=0;
DWORD g_IDThreadTvDisplay;
HWND g_hWndTvDisplay=0;

/////////////////////////////////
//optitrack 선언
using namespace CameraLibrary;

Camera *camera  ;
int cameraWidth ;
int cameraHeight;
IplImage* frame ;
IplImage* frame3c;
IplImage* showopti;
//////opengl 글자띄우기//////////
char s[30];
int font=(int)GLUT_BITMAP_8_BY_13;
///////////////////////////
//opti_cv글자 띄우기
char str_model_y[50]="3d model y_axis 0";
char str_obj1x[50]="1 x 0.00";
char str_obj1y[50]="1 y 0.00";
char str_obj2x[50]="2 x 0.00";
char str_obj2y[50]="2 y 0.00";
//
void initProgram();
void readSettingsFromINI();
void initCamera();
void getDepthFrame(bool bUseCal=0);
void getColorFrame();
void getDepthRanged();
void processFrame();
void exitProgram();
void exitCamera();
//void processKeyEvent(unsigned char key, int x, int y);
void Key(int key, int x, int y);

void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data);
void onNodeConnected();
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data);
void onColorNodeConnected();
DWORD WINAPI DSThread(void* arg);
void InitDS(bool bUseColor=0);
void exitDS();

void SetupRC(void);
void init(void);
void ChangeSize(int w, int h);
void ChangeSize2(int w, int h);
void RenderScene(void);
void Renderleft(void);
void Renderright(void);
void timer(int value);
void mouseButtonfunc(int button, int state, int x, int y);
void mouseWheel(int,int,int,int);
void myMotion(int x, int y);
void DrawDot(positionDot pt);
void setConfidence(int position);

void setCloseMode(int position);
void setFrameRate(int position);
void setDenoising(int position);
void setLight(int position);
void setCloseMode(int position);
void mouse_callback(int event, int x, int y, int flags, void* param);

void iplImageToDC(IplImage* pImgIpl, HDC hDC, CvRect rect);

void renderBitmapString(float x, float y, void *font,char *string);


bool g_bfff=1;

int ttt;
void main(int argc, char* argv[])
{

	CameraLibrary_EnableDevelopment();//optitrack 관련 
	CameraManager::X().WaitForInitialization();
	camera  = CameraManager::X().GetCamera();


	cameraWidth = camera->Width();
	cameraHeight = camera->Height();
	frame =cvCreateImage(cvSize(cameraWidth,cameraHeight),8,1);//------------

	glutInit(&argc, argv);
	initProgram();

	glutMainLoop();
	

}

void readSettingsFromINI()  // ini 파일에서 정보 읽어오는 함수
{
	TCHAR path[512];
	GetCurrentDirectory(512,path);  //프로젝트 경로
	wcscat(path,L"\\camera.ini");
	g_uFrameRate=GetPrivateProfileInt(TEXT("CameraSetting"),TEXT("FrameRate"),30,path);
	g_uConfidence=GetPrivateProfileInt(TEXT("CameraSetting"),TEXT("Confidence"),1,path);
	g_uLight=GetPrivateProfileInt(TEXT("CameraSetting"),TEXT("Light"),100,path);

	GetCurrentDirectory(512,path);  //프로젝트 경로
	wcscat(path,L"\\program.ini");
	g_uRangeMin=GetPrivateProfileInt(TEXT("DepthRange"),TEXT("Min"),-1,path);
	g_uRangeMax=GetPrivateProfileInt(TEXT("DepthRange"),TEXT("Max"),-1,path);
	g_xmove=GetPrivateProfileInt(TEXT("DepthRange"),TEXT("xmove"),-1,path); //89
	g_ymove=GetPrivateProfileInt(TEXT("DepthRange"),TEXT("ymove"),-1,path); //50
	g_xresize=GetPrivateProfileInt(TEXT("DepthRange"),TEXT("xresize"),-1,path);//119
	g_yresize=GetPrivateProfileInt(TEXT("DepthRange"),TEXT("yresize"),-1,path);//98
	g_Difference=GetPrivateProfileInt(TEXT("DepthRange"),TEXT("Difference"),-1,path);
}  

void initCamera()
{
	InitDS(true);
}

void initProgram() //프로그램 초기화 함수
{
	readSettingsFromINI();
	initCamera();
	
	g_imgDepth=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),IPL_DEPTH_16S,1);
	g_imgDepthRangedGray=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);
	g_imgColor=cvCreateImage(cvSize(640,480),8,3);
	g_imgColorcal=cvCreateImage(cvSize(640,480),8,3);
	g_colorcalspace=cvCreateImage(cvSize(640,480),8,3);
	g_memStorage=cvCreateMemStorage();
	g_imgBinary=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);
	g_bilateral=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);
	g_depthresize=cvCreateImage(cvSize(640,480),8,1);
	g_display=cvCreateImage(cvSize(640,480),8,3);
	g_display_left=cvCreateImage(cvSize(960,1080),8,3);
	g_display_right=cvCreateImage(cvSize(960,1080),8,3);
	g_display_stereo=cvCreateImage(cvSize(1920,1080),8,3);
	g_displayzoomin=cvCreateImage(cvSize(640,480),8,3);
	g_displayzoomleft=cvCreateImage(cvSize(640,400),8,3);
	g_displayzoomright=cvCreateImage(cvSize(640,400),8,3);
	g_tempdepth=cvCreateImage(cvSize(640,480),8,3);
	g_imgColorcal = cvCreateImage(cvSize(g_imgColor->width - g_xresize ,g_imgColor->height - g_yresize),8,3);

	cvNamedWindow("Control",1); //trackbar 관련 설정 
	cvResizeWindow("Control",800,430);
	
	cvCreateTrackbar("Confidence","Control",(int*)&g_uConfidence,9999,setConfidence);
	cvCreateTrackbar("RangeMin","Control",&g_uRangeMin,g_uMaxDepth,0);
	cvCreateTrackbar("RangeMax","Control",&g_uRangeMax,g_uMaxDepth,0);

	cvCreateTrackbar("Difference","Control",&g_Difference,100,0);
	cvCreateTrackbar("Color","Control",&g_colorenalbe,1,0);
	cvCreateTrackbar("Filling","Control",&g_holefilling,1,0);
	cvCreateTrackbar("Optitrack","Control",&g_optitrack,1,0);

	cvCreateTrackbar("Xmove","Control",&g_xmove,320,0);
	cvCreateTrackbar("Ymove","Control",&g_ymove,240,0);

	cvCreateTrackbar("Xresize","Control",&g_xresize,200,0);
	cvCreateTrackbar("Yresize","Control",&g_yresize,200,0);//------------

	CameraLibrary_EnableDevelopment();//optitrack
	CameraManager::X().WaitForInitialization();
	camera->SetVideoType(SegmentMode);
	camera->SetFrameRate(120);
	camera->SetExposure(40);
	camera->SetThreshold(150);
	camera->SetIntensity(35);
	camera->Start();

	frame=cvCreateImage(cvSize(cameraWidth,cameraHeight),8,1);
	frame3c=cvCreateImage(cvSize(cameraWidth,cameraHeight),8,3);
    showopti=cvCreateImage(cvSize(400,300),8,3);///-------------------------------------

	
	init();//opengl
	
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB );
	glutInitWindowSize(640,480);

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glEnable(GL_DEPTH_TEST);

	windowleft=glutCreateWindow("left3d"); //왼쪽 glutwindow 관련 설정
	glutMouseFunc(mouseButtonfunc);
	glutMouseWheelFunc(mouseWheel);
	glutMotionFunc(myMotion); 
	glutSpecialFunc(Key);
	glutSetWindow(windowleft);
	glLoadIdentity();
	glutReshapeFunc(ChangeSize);
	glutSetWindow(windowleft);
	glutDisplayFunc(Renderleft);
	glEnable(GL_DEPTH_TEST);//깊이 따라서 모델 배열 및 그려주기
	glEnable(GL_CULL_FACE);// 폴리곤 추출하기(보이지 않는면 안그리기)

	windowright=glutCreateWindow("right3d"); //오른쪽 glutwindow 관련 설정
	glutMouseFunc(mouseButtonfunc);
	glutMouseWheelFunc(mouseWheel);
	glutMotionFunc(myMotion); 
	glutSpecialFunc(Key);
	glutSetWindow(windowright);
	glLoadIdentity();
	glutReshapeFunc(ChangeSize2);
	glutSetWindow(windowright);
	glutDisplayFunc(Renderright);
	glEnable(GL_DEPTH_TEST);//깊이 따라서 모델 배열 및 그려주기
	glEnable(GL_CULL_FACE);// 폴리곤 추출하기(보이지 않는면 안그리기)//------------
	g_hThreadTvDisplay=CreateThread(NULL,0,CreateStereoWindowsAPI, L"TVstereo",0,&g_IDThreadTvDisplay);

}

void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
	EnterCriticalSection(&g_csDepthFrame);
	g_imgDepthBuffer->imageData=(char*)(const signed short*)(data.depthMap);

	if(g_imgColorBuffer)
	{
		const UV* uvmap=data.uvMap;
		EnterCriticalSection(&g_csColorFrame);
		EnterCriticalSection(&g_csCalibrationFrame);
		cvZero(g_imgCalBuffer);
		for(int x=0;x<g_imgCalBuffer->width;x++)
		{
			for(int y=0;y<g_imgCalBuffer->height;y++)
			{
				float u=uvmap[y*g_imgCalBuffer->width+x].u;
				float v=uvmap[y*g_imgCalBuffer->width+x].v;
				if(abs(u)!=FLT_MAX)
				{
					int x_=g_uColorWidthDS*u;
					int y_=g_uColorHeightDS*v;
					uchar* dst=GET2D8U3CH(g_imgCalBuffer,x,y);
					uchar* src=GET2D8U3CH(g_imgColorBuffer,x_,y_);
					dst[0]=src[0];
					dst[1]=src[1];
					dst[2]=src[2];
				}
			}
		}
		LeaveCriticalSection(&g_csColorFrame);
		LeaveCriticalSection(&g_csCalibrationFrame);
	}
	
	g_bFlagNewFrame=true;
	LeaveCriticalSection(&g_csDepthFrame);	
	if(g_bClose)
		g_Context.quit();
}

void onNodeConnected()
{
	g_uMaxDepth=g_DNode.getRange()*1000;
	if(g_uRangeMax==-1)
		g_uRangeMax=g_uMaxDepth;
	g_uMinDepth=0;
	if(g_uRangeMin==-1)
		g_uRangeMin=g_uMinDepth;
	g_DNode.newSampleReceivedEvent().connect(&onNewDepthSample);
	g_DNode.setEnableDepthMap(true);
	g_DNode.setEnableUvMap(true);//
	DepthNode::Configuration config = g_DNode.getConfiguration();
	FrameFormat_toResolution(config.frameFormat,&g_uWidthDS,&g_uHeightDS);
	g_imgDepthBuffer=cvCreateImageHeader(cvSize(g_uWidthDS,g_uHeightDS),IPL_DEPTH_16S,1);
	g_imgCalBuffer=cvCreateImage(cvSize(g_uWidthDS,g_uHeightDS), IPL_DEPTH_8U, 3);

	config.framerate = g_uFrameRate;
	DepthSense::Device::Model model=g_Context.getDevices()[0].getModel();
	config.mode = g_bIsCloseMode||
		( model ==DepthSense::Device::Model::MODEL_DS325  || model ==DepthSense::Device::Model::MODEL_VF0780) ? 
		DepthNode::CAMERA_MODE_CLOSE_MODE:DepthNode::CAMERA_MODE_LONG_RANGE;
	config.saturation = true;

	try 
	{
		g_Context.requestControl(g_DNode,0);
		g_DNode.setConfiguration(config);
		g_DNode.setConfidenceThreshold(g_uConfidence);
		g_DNode.setEnableDenoising(g_bDenoising);
		if( model !=DepthSense::Device::Model::MODEL_DS325  && model !=DepthSense::Device::Model::MODEL_VF0780){
			g_DNode.setIlluminationLevel(g_uLight);	
		}
	}
	catch (Exception& e)
	{	
	}
	g_Context.releaseControl(g_DNode);
}

void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{
	EnterCriticalSection(&g_csColorFrame);

	int k = 0;
	for(int y = 0; y < g_imgY->height; y++)
	{
		for(int x = 0; x < g_imgY->width; x++)
		{
			GET2D8U(g_imgY,x,y)=data.colorMap[2 * k++];
		}
	}
	k=0;
	for(int y = 0; y < g_imgCb->height; y++)
	{
		for(int x = 0; x < g_imgCb->width/2; x++)
		{
			GET2D8U(g_imgCb,x*2+1,y)=GET2D8U(g_imgCb,x*2,y)=data.colorMap[k*4+1];
			GET2D8U(g_imgCr,x*2+1,y)=GET2D8U(g_imgCr,x*2,y)=data.colorMap[k++*4+3];
		}
	}

	cvMerge(g_imgY, g_imgCr, g_imgCb, NULL, g_imgColorBuffer);
	cvCvtColor(g_imgColorBuffer, g_imgColorBuffer, CV_YCrCb2BGR);
	g_bFlagNewColorFrame=true;

	LeaveCriticalSection(&g_csColorFrame);	
	if(g_bClose)
		g_Context.quit();
}

void onColorNodeConnected()
{
	g_CNode.newSampleReceivedEvent().connect(&onNewColorSample);
	g_CNode.setEnableColorMap(true);
	ColorNode::Configuration config = g_CNode.getConfiguration();
	FrameFormat_toResolution(config.frameFormat,&g_uColorWidthDS,&g_uColorHeightDS);

	g_imgColorBuffer=cvCreateImage(cvSize(g_uColorWidthDS,g_uColorHeightDS), IPL_DEPTH_8U, 3);

	g_imgY = cvCreateImage(cvSize(g_uColorWidthDS,g_uColorHeightDS), IPL_DEPTH_8U, 1);
	g_imgCb = cvCreateImage(cvSize(g_uColorWidthDS,g_uColorHeightDS), IPL_DEPTH_8U, 1);
	g_imgCr = cvCreateImage(cvSize(g_uColorWidthDS,g_uColorHeightDS), IPL_DEPTH_8U, 1);

	try 
	{
		g_Context.requestControl(g_CNode,0);
		g_CNode.setConfiguration(config);
	}
	catch (Exception& e)
	{	
	}
	g_Context.releaseControl(g_CNode);
}

void getDepthFrame(bool bUseCal)
{
	while(1)
	{
		EnterCriticalSection(&g_csDepthFrame);
		if(g_bFlagNewFrame)
		{
			cvResize(g_imgDepthBuffer,g_imgDepth,0);
			g_bFlagNewFrame=false;
			LeaveCriticalSection(&g_csDepthFrame);
			cvFlip(g_imgDepth,g_imgDepth,1);
			if(bUseCal)
			{
				EnterCriticalSection(&g_csCalibrationFrame);
				cvResize(g_imgCalBuffer,g_imgCalibration,0);
				LeaveCriticalSection(&g_csCalibrationFrame);
				cvFlip(g_imgCalibration,g_imgCalibration,1);
			}
			break;
		}
		LeaveCriticalSection(&g_csDepthFrame);
	}

}

void getColorFrame()
{
	while(1)
	{
		EnterCriticalSection(&g_csColorFrame);
		if(g_bFlagNewColorFrame)
		{
			cvResize(g_imgColorBuffer,g_imgColor,0);
			g_bFlagNewColorFrame=false;
			LeaveCriticalSection(&g_csColorFrame);
			cvFlip(g_imgColor,g_imgColor,1);
			break;
		}
		LeaveCriticalSection(&g_csColorFrame);
	}
}

DWORD WINAPI DSThread(void* arg)
{
	g_Context.run();

	DWORD  exitcode; 
	GetExitCodeThread(g_hThreadDepth, &exitcode);
	g_bClose=false;
	ExitThread(exitcode);
	CloseHandle(g_hThreadDepth);
}

void InitDS(bool bUseColor)
{
	InitializeCriticalSection(&g_csDepthFrame);
	g_Context = Context::create("localhost");
	vector<Device> da = g_Context.getDevices();

	if (da.size() >= 1)
	{
		vector<Node> nodes = da[0].getNodes();
		for (int n = 0; n < (int)nodes.size();n++)
		{
			if(nodes[n].is<DepthNode>())
			{
				g_DNode = nodes[n].as<DepthNode>();
				onNodeConnected();
				g_Context.registerNode(nodes[n]);

				if(!bUseColor)
					break;
			}
			else if(nodes[n].is<ColorNode>() && bUseColor)
			{
				g_imgCalibration=cvCreateImage(cvSize(COLOR_WIDTH,COLOR_HEIGHT),8,3); //
				InitializeCriticalSection(&g_csColorFrame);
				InitializeCriticalSection(&g_csCalibrationFrame);
				g_CNode = nodes[n].as<ColorNode>();
				onColorNodeConnected();
				g_Context.registerNode(nodes[n]);
			}
		}
		g_Context.startNodes();
		g_hThreadDepth=CreateThread(NULL,0,DSThread, 0,0,&g_IDThreadDepth);
	}
}

void exitDS()
{
	g_bClose=true;
	while(g_bClose);

	DeleteCriticalSection(&g_csDepthFrame);

	cvReleaseImage(&g_imgDepthBuffer);

	if(g_imgColorBuffer)
	{
		DeleteCriticalSection(&g_csColorFrame);
		DeleteCriticalSection(&g_csCalibrationFrame);
		cvReleaseImage(&g_imgColorBuffer);
		cvReleaseImage(&g_imgY);
		cvReleaseImage(&g_imgCb);
		cvReleaseImage(&g_imgCr);
		cvReleaseImage(&g_imgCalBuffer);
		
	}

	g_Context.stopNodes();
	if (g_DNode.isSet())
		g_Context.unregisterNode(g_DNode);

	if (g_CNode.isSet())
		g_Context.unregisterNode(g_CNode);
}

void getDepthRanged()
{
	for(int y=0;y<DEPTH_HEIGHT;y++)
	{
		for(int x=0;x<DEPTH_WIDTH;x++)
		{
			short val=GET2D16S(g_imgDepth,x,y);
			if(val && g_uRangeMin<=val &&  val<=g_uRangeMax)
			{
				GET2D8U(g_imgDepthRangedGray,x,y)=255-(int) ( (float)(val-g_uRangeMin)/(g_uRangeMax-g_uRangeMin)*254 );
			}
			else
				GET2D8U(g_imgDepthRangedGray,x,y)=0;
		}
	}

	if(g_holefilling==0){

		cvCopy(g_imgDepthRangedGray,g_bilateral);
		cvResize(g_bilateral,g_depthresize,1);

	}

	else{

		cvSmooth(g_imgDepthRangedGray,g_imgDepthRangedGray,CV_MEDIAN,3,3);
		cvSmooth(g_imgDepthRangedGray,g_bilateral,CV_BILATERAL,3,3);
		cvSmooth(g_bilateral,g_bilateral,CV_MEDIAN,3,3);
		cvResize(g_bilateral,g_depthresize,CV_INTER_CUBIC);
		cvSmooth(g_depthresize,g_depthresize,CV_MEDIAN,3,3);
	}

}

void processFrame()
{
	cvShowImage("Depthmap",g_bilateral);
	cvShowImage("Color",g_imgColor);
	
}


void getBinary()
{
	cvThreshold(g_imgDepthRangedGray,g_imgBinary,g_uThreshold_max,255,CV_THRESH_TOZERO_INV);
	cvThreshold(g_imgBinary,g_imgBinary,g_uThreshold_min,255,CV_THRESH_BINARY);
}

void setCloseMode(int position)
{
	DepthNode::Configuration config = g_DNode.getConfiguration();
	config.mode = g_bIsCloseMode ? 
		DepthNode::CAMERA_MODE_CLOSE_MODE:DepthNode::CAMERA_MODE_LONG_RANGE;

	try 
	{
		g_Context.requestControl(g_DNode,0);
		g_DNode.setConfiguration(config);
	}
	catch(Exception &e)
	{
	}
	g_Context.releaseControl(g_DNode);
}

void setLight(int position)
{
	DepthSense::Device::Model model=g_Context.getDevices()[0].getModel();
	if( model !=DepthSense::Device::Model::MODEL_DS325  && model !=DepthSense::Device::Model::MODEL_VF0780)
	{
		try 
		{
			g_Context.requestControl(g_DNode,0);
			g_DNode.setIlluminationLevel(g_uLight);
		}
		catch(Exception &e)
		{
		}
		g_Context.releaseControl(g_DNode);
	}
}

void setDenoising(int position)
{
	try 
	{
		g_Context.requestControl(g_DNode,0);
		g_DNode.setEnableDenoising(g_bDenoising);
	}
	catch(Exception &e)
	{
	}
	g_Context.releaseControl(g_DNode);
}

void setFrameRate(int position)
{
	DepthNode::Configuration config = g_DNode.getConfiguration();
	config.framerate=g_uFrameRate;

	try 
	{
		g_Context.requestControl(g_DNode,0);
		g_DNode.setConfiguration(config);
	}
	catch(Exception &e)
	{
	}
	g_Context.releaseControl(g_DNode);
}

void setConfidence(int position)
{
	try 
	{
		g_Context.requestControl(g_DNode,0);
		g_DNode.setConfidenceThreshold(g_uConfidence);
	}
	catch(Exception &e)
	{
	}
	g_Context.releaseControl(g_DNode);
}

void mouseButtonfunc(int button, int state, int x, int y)
{
	if(button==GLUT_LEFT_BUTTON && state ==GLUT_DOWN) {
		mouseState=state;
		mouseButton=button;
		mouseX=x;
		mouseY=y;
	}
	else if(button==GLUT_LEFT_BUTTON && state == GLUT_UP) {
		mouseState=-1;
	}
	else if(button==GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
		mouseState=state;
		mouseButton=button;
		mouseX=x;
		mouseY=y;
	}
	else if(button==GLUT_RIGHT_BUTTON && state == GLUT_UP) {
		mouseState=-1;
	}
	else if(button==GLUT_MIDDLE_BUTTON && state == GLUT_DOWN) {
		xTheta=yTheta=0.0;
		scale=1.0;
	}
	else return;

	glutPostRedisplay();
}



void mouseWheel(int button, int dir, int x, int y)
{ 
	
	if (dir > 0)
	{ 
		if(g>3){
			g=2;
		}
		if(g<1){
			g=1;
		}
		g++;

		printf("Zoom in\n",1);
		if(g==2){
			glutSetWindow(windowleft);
		    glViewport(-110, -55, 960, 720);
			glOrtho(-320,320, -240, 240, -1000, 1000);


			glutSetWindow(windowright);
			glViewport(-110, -55, 960, 720);
			glOrtho(-320,320, -240, 240, -1000, 1000);

			

		}
		if(g==3){
			glutSetWindow(windowleft);
			glViewport(-230, -110, 1280, 960);
			glOrtho(-320,320, -240, 240, -1000, 1000);

			glutSetWindow(windowright);
			glViewport(-230, -110, 1280, 960);
			glOrtho(-320,320, -240, 240, -1000, 1000);


		}
		// Zoom in
	}
	else
	{  
		if( g>2){
			g=3;
		}
		g--;
		printf("Zoom out!\n",2);
		if(g==2){
			glutSetWindow(windowleft);
			glViewport(-110, -55, 960, 720);
			glOrtho(-320,320, -240, 240, -1000, 1000);


			glutSetWindow(windowright);
			glViewport(-110, -55, 960, 720);
			glOrtho(-320,320, -240, 240, -1000, 1000);

		

		}
		if(g==1){
			glutSetWindow(windowleft);
			glViewport(1, 1, 640, 480);
			glOrtho(-320,320, -240, 240, -1000, 1000);


			glutSetWindow(windowright);
			glViewport(1, 1, 640, 480);
			glOrtho(-320,320, -240, 240, -1000, 1000);

		

		}
		// Zoom out
	}

	return;
}

void myMotion(int x, int y)
{
	if(mouseButton == GLUT_LEFT_BUTTON && mouseState == GLUT_DOWN) 
	{
		yTheta -= (GLfloat)(mouseX - x);
		xTheta -= (GLfloat)(mouseY - y);
	}
	else if(mouseButton == GLUT_RIGHT_BUTTON && mouseState == GLUT_DOWN) 
	{
		if(mouseY!=y) 
			scale = scale * pow((double)scaleDelta, (double)(mouseY - y));
	}
	else return;

	mouseX = x;
	mouseY = y;
	glutPostRedisplay();
}

void exitProgram()
{
	exitCamera();
	cvDestroyAllWindows();
	cvReleaseImage(&g_imgDepth);
	cvReleaseImage(&g_imgDepthRangedGray);
	cvReleaseImage(&g_bilateral);
	cvReleaseImage(&g_imgColor);
	cvReleaseImage(&g_display);
	cvReleaseImage(&g_displayzoomin);
	cvReleaseImage(&g_displayzoomleft);
	cvReleaseImage(&g_displayzoomright);
	cvReleaseImage(&g_display_left);
	cvReleaseImage(&g_display_right);
	cvReleaseImage(&g_display_stereo);
	cvReleaseMemStorage(&g_memStorage);
	cvReleaseImage(&g_imgColorcal);	
	cvReleaseImage(&g_colorcalspace);
	cvReleaseImage(&g_tempdepth);
	cvReleaseImage(&g_depthresize);
	//optitrack
	cvReleaseImage(&frame);
	cvReleaseImage(&frame3c);
	cvReleaseImage(&showopti);
	camera->Stop();
	camera->Release();
	CameraManager::X().Shutdown();
	//
	exit(0);
}

void exitCamera()
{
	exitDS();
}


//////////////////////////////////////



int a=0; //죽는 위치 찾는 방법


void init(void) //opengl window창 초기화 
{   
	int cnt=0;

	rotate_x = 1;
	rotate_y = 0;
	rotate_z = 0;

	direction = 1;

	rt = false;

	theta = 0.0f;


	glutTimerFunc(1000/30, timer, 1);

}

void ChangeSize(int w, int h) //왼쪽 glut window 설정
{
	if( h == 0 )
		h = 1;

	glutSetWindow(windowleft);
	glViewport(0,0, 640, 480);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();


	glOrtho(-320,320, -240, 240, -1000, 1000);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void ChangeSize2(int w, int h) // 오른쪽 glut window 설정
{
	if( h == 0 )
		h = 1;

	glutSetWindow(windowright);

	glViewport(0,0, 640, 480);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();


	glOrtho(-320,320, -240, 240, -1000, 1000);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}


void timer(int value)
{   //ttt=GetTickCount();
	if(rt)

		theta += 2.0;

	getDepthFrame(true);
	getDepthRanged();
	getBinary();
	getColorFrame();

	free(p);
	p = new positionDot[2000000];

	int addc=1;
	int addc3=1;
	int addc4=1;
	int addc2=1;

	cvResize(g_imgColor,g_imgColorcal,CV_INTER_CUBIC);

	cvMerge(g_depthresize,g_depthresize,g_depthresize,NULL,g_tempdepth);
	cvCopy(g_tempdepth,g_colorcalspace);
	
	

	for( int x = g_xmove ; x< g_tempdepth->width ; x++)
	{
	for( int y = g_ymove ; y< g_tempdepth->height ; y++)
		{
		

			if(x < g_imgColorcal->width+g_xmove && y <g_imgColorcal->height+g_ymove)
			{
				
				GET2D8U3CH(g_imgColorcal,x-g_xmove,y-g_ymove)[0]= GET2D8U3CH(g_imgColorcal,x-g_xmove,y-g_ymove)[0]*0.9 + GET2D8U3CH(g_tempdepth,x,y)[0]*0.1;
				GET2D8U3CH(g_imgColorcal,x-g_xmove,y-g_ymove)[1]= GET2D8U3CH(g_imgColorcal,x-g_xmove,y-g_ymove)[1]*0.9 + GET2D8U3CH(g_tempdepth,x,y)[1]*0.1;
				GET2D8U3CH(g_imgColorcal,x-g_xmove,y-g_ymove)[2]= GET2D8U3CH(g_imgColorcal,x-g_xmove,y-g_ymove)[2]*0.9 + GET2D8U3CH(g_tempdepth,x,y)[2]*0.1;

				GET2D8U3CH(g_colorcalspace,x,y)[0]=GET2D8U3CH(g_imgColorcal,x-g_xmove,y-g_ymove)[0];
				GET2D8U3CH(g_colorcalspace,x,y)[1]=GET2D8U3CH(g_imgColorcal,x-g_xmove,y-g_ymove)[1];
				GET2D8U3CH(g_colorcalspace,x,y)[2]=GET2D8U3CH(g_imgColorcal,x-g_xmove,y-g_ymove)[2];
			}
		}
	}


	if(g_optitrack==0){   //optitrack----------------------------------------------------------

	}
	else{
		Frame *raw_frame = camera->GetLatestFrame();
		if(raw_frame)
		{
			int optivalue[10]={0};
			int optiyval[10]={0};
			int cntcnt=0;
			int midxvalue=0;
			int ytheta3d=0;
			raw_frame->Rasterize(cameraWidth,cameraHeight,cameraWidth,8,frame->imageData);

			cvMerge(frame,frame,frame,NULL,frame3c);
	

			for(int i=0;i<raw_frame->ObjectCount();i++)
			{
				cObject *obj=raw_frame->Object(i);
				if(obj->Radius()>4){

					optivalue[cntcnt]=obj->X();
					optiyval[cntcnt]=obj->Y();
					
				
					cvDrawCircle(frame3c, cvPoint(obj->X(),obj->Y()), obj->Radius(),CV_RGB(255,255,255),30);
					
					
					cntcnt++;
				}

			}
			for(int i=0; i<cntcnt;i++ ){
			
				midxvalue= (midxvalue + optivalue[i])/(i+1);
			}

			if(midxvalue>0){
				yTheta=(midxvalue/10)-70;
				ytheta3d=yTheta;
			}	

			cvResize(frame3c,showopti,CV_INTER_CUBIC);

			sprintf_s(str_model_y,"3d model y_axis %d",ytheta3d);

			sprintf_s(str_obj1x,"1_x %d",optivalue[0]);
			sprintf_s(str_obj1y,"1_y %d",optiyval[0]);
			sprintf_s(str_obj2x,"2_x %d",optivalue[1]);
			sprintf_s(str_obj2y,"2_y %d",optiyval[1]);

			cvPutText(showopti,str_model_y,cvPoint(10,20),&cvFont(1,1),CV_RGB(255, 255, 255));

			cvPutText(showopti,str_obj1x,cvPoint(10,35),&cvFont(1,1),CV_RGB(255, 255, 255));
			cvPutText(showopti,str_obj1y,cvPoint(10,50),&cvFont(1,1),CV_RGB(255, 255, 255));
			cvPutText(showopti,str_obj2x,cvPoint(10,65),&cvFont(1,1),CV_RGB(255, 255, 255));
			cvPutText(showopti,str_obj2y,cvPoint(10,80),&cvFont(1,1),CV_RGB(255, 255, 255));

			cvShowImage("사용자 위치",showopti);

			raw_frame->Release();

		}
		else{
			yTheta=yTheta;
		}
	}//------------------------------------------------------------------------------------//

	if( g_depthresize != NULL)
	{
		a++;
		cnt=0;
		g_dtotal=0;
		g_dtotal2=0;
		addc=1;
		addc2=1;
		g_temp=0;
		g_temp2=0;

		g_dvmax=0;
		g_dvmin=255;
		g_dvmid=0;
		
		for(int j = 0; j < g_colorcalspace->height; j++)
		{         
			for(int i = 0; i < g_colorcalspace->width; i++)
			{

				if(GET2D8U3CH(g_colorcalspace,i,j)[0]!=0 && GET2D8U3CH(g_colorcalspace,i,j)[1]!=0 && GET2D8U3CH(g_colorcalspace,i,j)[2] != 0){
					if(GET2D8U(g_depthresize,i,j)>0&&GET2D8U(g_depthresize,i,j)<=255)
					{
						p[cnt].x = i;
						p[cnt].y = j;

						p[cnt].z = (float)GET2D8U(g_depthresize,i,j); // 깊이 값

						p[cnt].b= GET2D8U3CH(g_colorcalspace,i,j)[0]; //bgr  값
						p[cnt].g= GET2D8U3CH(g_colorcalspace,i,j)[1]; 
						p[cnt].r= GET2D8U3CH(g_colorcalspace,i,j)[2];
						p[cnt].g_tv=true;
						p[cnt].m_z=0;

						cnt++;

						cnt_points=cnt;			


					}
				}

			}
		}	

		

		if(cnt==0){
			cnt=1;  //촬영 범위 안에 대상이 없으면 count 1로 초기화
		}


		//깊이 값 전체 평균 구하기
		for(int i=1;i<cnt;i++){
			g_dvmid=g_dvmid+p[i].z;
		}
		g_dvmid=g_dvmid/cnt;
		g_dvmid2=g_dvmid;

		//최근값 구하기
		for(int i=0;i<cnt; i++){	
			g_temp=p[i].z;
			if(g_temp>g_dvmax){
				g_dvmax=g_temp;
				g_dvmax2=g_dvmax;
			}
		}


		//최원값 구하기
		for(int i=0;i<cnt;i++){
			g_temp2=p[i].z;

			if(g_temp2<g_dvmin){
				g_dvmin=g_temp2;
			}
		}
		//최원값 조절
		if(g_dvmin<g_dvmid-50){
			g_dvmin=g_dvmid-50;
		}
	
			g_dtotal=cnt;

	
		//경계선 부분 매끄럽게
		if(g_dtotal<30){
			// 객체가 없을 때는 그냥 넘어가기
		}
		else if(g_holefilling==0){//holefilling 미적용
		
		}
		else { //holefilling 조건 넣기
		//	int ttt=GetTickCount();// 걸리는 시간 재기 단위는 millisec
			for(int i=1; i<g_dtotal; i++){
				p[i].m_z=0;
				if(i<30){
					for(int j=0; j<60;j++){
						p[i].m_z=p[i].m_z+p[j].z;
					}
					p[i].m_z=(p[i].m_z)/60;
				}
				else if(30<=i&&i<=g_dtotal-30){

					for(int j=0; j<60;j++){
						p[i].m_z=p[i].m_z+p[(i-30)+j].z;
					}
					p[i].m_z=(p[i].m_z)/60;
				}
				else if(i>g_dtotal-30){
					for(int j=0;j<60;j++){
						p[i].m_z=p[i].m_z+p[(i-60)+j].z;
					}
					p[i].m_z=(p[i].m_z)/60;
				}

			}


			for(int i=1; i<g_dtotal; i++){
				if(p[i].z<=p[i].m_z)
				{
					int k= p[i].m_z-p[i].z;

					if(0<k&&p[i].z<200){
						p[i].z=p[i].z+k;
					}
					if(k>=7){//평균보다 뒤에 있을 때 
						p[i].g_tv=false;
						//p[i].z=p[i].z+k;
					}

					else if(k<0){ //평균보다 앞에 있을때
						p[i].z=p[i].z-k;
					}	

				}
			} 
		
			for(int i=1; i<g_dtotal; i++){ //누실된 point 복구 및 채우기
				if(p[i].z>0&&p[i].z<g_dvmax){
				
						for(int j=0; j<=6; j++){
							p[g_dtotal+addc2].x=p[i].x;
							p[g_dtotal+addc2].y=p[i].y;
							p[g_dtotal+addc2].b=p[i].b;
							p[g_dtotal+addc2].g=p[i].g;
							p[g_dtotal+addc2].r=p[i].r;	
							p[g_dtotal+addc2].z=p[i].z+j;
							p[g_dtotal+addc2].g_tv=true;
							addc2++;

						}

						for(int j=1; j<6; j++){
							if(p[g_dtotal+addc2-j].z==p[i].z){
								p[g_dtotal+addc2-j].g_tv=false;
							}						
						}									
				}

			}

			g_dtotal=g_dtotal+addc2;//----------------------------------------------
		}

		}

	processFrame();
	//cvWaitKey(1);
	glutTimerFunc(1000/60, timer, 1);
	glutPostRedisplay();

}

void renderBitmapString(float x, float y, void *font,char *string)//opengl 위에 글씨그리기
{
  
  char *c;
  // set position to start drawing fonts
  glRasterPos2f(x, y);
  // loop all the characters in the string
  for (c=string; *c != '\0'; c++) {
    glutBitmapCharacter(font, *c);
  }
} //-----------------------------------

void Renderleft(void){ //왼쪽 glut window를 그려주는 부분

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glRotatef(xTheta, 1.0, 0.0, 0.0);    
	glRotatef(yTheta-g_Difference*0.1, 0.0, 1.0, 0.0);

	if(g_holefilling==0){//holefilling 미적용
		glPointSize(1);
	}
	else {
		glPointSize(2.2);
	}
	glBegin(GL_POINTS);
	// #pragma omp parallel for
	for(int i = 0; i <g_dtotal; i++)
	{
		if(p[i].g_tv==true){
			DrawDot(p[i]);
			g_dtotal2++;
		}

	}	

	glEnd();

	glutSwapBuffers();
	//printf("Rendering time: %d\n",GetTickCount()-ttt);

	glutPostWindowRedisplay(windowright);//2
	glReadPixels(0,80,640,400,GL_BGR_EXT,GL_UNSIGNED_BYTE,g_displayzoomleft->imageData);
	cvResize(g_displayzoomleft, g_display_left, CV_INTER_CUBIC );
	cvFlip(g_display_left, NULL, 0);

}

void Renderright(void){ //오른쪽 glut window를 그려주는 부분 
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glRotatef(xTheta, 1.0, 0.0, 0.0);    
	glRotatef(yTheta+g_Difference*0.1, 0.0, 1.0, 0.0); //90
	
	if(g_holefilling==0){//holefilling 미적용
		glPointSize(1);
	}
	else {
		glPointSize(2.2);
	}
	glBegin(GL_POINTS);
	for(int i = 0; i <g_dtotal; i++)
	{
		if(p[i].g_tv==true){
			DrawDot(p[i]);
		}

	}
	glEnd();
	glutSwapBuffers();

	glutPostWindowRedisplay(windowleft); //1
	glReadPixels(0,80,640,400,GL_BGR_EXT,GL_UNSIGNED_BYTE,g_displayzoomright->imageData);
	cvResize(g_displayzoomright, g_display_right, CV_INTER_CUBIC );
	cvFlip(g_display_right, NULL, 0);
	

	cvSetImageROI(g_display_stereo,cvRect(0,0,960,1080));
	cvResize(g_display_left,g_display_stereo);// 리사이즈 함수
	
	cvSetImageROI(g_display_stereo,cvRect(960,0,960,1080));// 영역 구하기

	cvResize(g_display_right,g_display_stereo);// 다시 리사이즈

	iplImageToDC(g_display_stereo,GetDC(g_hWndTvDisplay),cvRect(0,0,1920,1080));



}

//win api consol 창의 이미지를 복사해서 전체화면으로 보여주는 함수
DWORD __stdcall CreateStereoWindowsAPI(void* lpszClass)
{
 DWORD  exitcode; 
 HINSTANCE g_Inst;
 HWND hWnd;
 MSG Message;
 WNDCLASS WndClass;
 HINSTANCE hInstance = GetModuleHandle(NULL);
 g_Inst = hInstance;
 WndClass.cbClsExtra = 0;
 WndClass.cbWndExtra = 0;
 WndClass.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
 WndClass.hCursor = LoadCursor(NULL, IDC_ARROW);
 WndClass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
 WndClass.hInstance = hInstance;
 WndClass.lpszClassName = (LPCWSTR)lpszClass;
 WndClass.lpszMenuName = NULL;
 WndClass.style = CS_HREDRAW|CS_VREDRAW;
 WndClass.lpfnWndProc = WndProc;
 RegisterClass(&WndClass);
 g_hWndTvDisplay = CreateWindowW((LPCWSTR)lpszClass, (LPCWSTR)lpszClass, WS_POPUP|WS_BORDER, 
  1920, 0, 1920, 1080,
  NULL, (HMENU)NULL, hInstance, NULL);
  
 
ShowWindow(g_hWndTvDisplay, SW_SHOW);
 while(GetMessage(&Message, NULL, 0, 0))
 {
  TranslateMessage(&Message);
  DispatchMessage(&Message);
 }
 GetExitCodeThread(g_hThreadTvDisplay, &exitcode);
 ExitThread(exitcode);
 CloseHandle(g_hThreadTvDisplay);
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT iMessage, WPARAM wParam, LPARAM IParam)
{
 switch(iMessage)
 {
 case WM_DESTROY:
  PostQuitMessage(0);
  return 0;
 }
 return(DefWindowProc(hWnd, iMessage, wParam, IParam));
} 

void iplImageToDC(IplImage* pImgIpl, HDC hDC, CvRect rect)  //IplImage를 dc에 그리는 함수
{
 BITMAPINFO bitmapInfo;
 bitmapInfo.bmiHeader.biSize=sizeof(BITMAPINFOHEADER);
 bitmapInfo.bmiHeader.biPlanes=1;
 bitmapInfo.bmiHeader.biCompression=BI_RGB;
 bitmapInfo.bmiHeader.biXPelsPerMeter=100;
 bitmapInfo.bmiHeader.biYPelsPerMeter=100;
 bitmapInfo.bmiHeader.biClrUsed=0;
 bitmapInfo.bmiHeader.biClrImportant=0;
 bitmapInfo.bmiHeader.biSizeImage=0;
 bitmapInfo.bmiHeader.biWidth=pImgIpl->width;
 bitmapInfo.bmiHeader.biHeight=-pImgIpl->height;
 IplImage* tempImage;
 if(pImgIpl->nChannels == 3)
 {
  tempImage = (IplImage*)cvClone(pImgIpl);
  bitmapInfo.bmiHeader.biBitCount=tempImage->depth * tempImage->nChannels;
 }
 else if(pImgIpl->nChannels == 1)
 {
  tempImage =  cvCreateImage(cvGetSize(pImgIpl), IPL_DEPTH_8U, 3);
  cvCvtColor(pImgIpl, tempImage, CV_GRAY2BGR);
  bitmapInfo.bmiHeader.biBitCount=tempImage->depth * tempImage->nChannels;
 }
 ::SetStretchBltMode(hDC,COLORONCOLOR);
 ::StretchDIBits(hDC, rect.x, rect.y, rect.width, rect.height, 
  0, 0, tempImage->width, tempImage->height, tempImage->imageData, &bitmapInfo, 
  DIB_RGB_COLORS, SRCCOPY);
 cvReleaseImage(&tempImage);
}
//-----------------------------------------------------------------------------
void Key(int key, int x, int y){ // 키보드 이벤트

	switch(key){

	case GLUT_KEY_LEFT :
		yTheta -=10;
		break;
	case GLUT_KEY_RIGHT : 
		yTheta +=10;
		break;
	case GLUT_KEY_END:
		exitProgram();
		break;
	default :
		break;
	}
	glutPostRedisplay();
}


void DrawDot(positionDot pt)//실질적으로 받아오는 좌표 수정하여 그려주는 함수
{
	float x, y, z, b, g, r;

	x = (GLfloat)(pt.x-300);
	y = -(GLfloat)(pt.y-230);
	z = (pt.z);
	b =  pt.b/255.0;
	g =  pt.g/255.0;
	r =  pt.r/255.0;

	if(g_colorenalbe==0){
	glColor3f(pt.z/255,pt.z/255,pt.z/255);//회색 깊이 값으로 보기
	}
	else
	{
	glColor3f(r,g,b);//r,g,b
	}
	glVertex3f(x, y, z); //x,y,z

}
