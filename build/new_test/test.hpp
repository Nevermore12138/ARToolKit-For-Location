#include <stdio.h>

#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#  define snprintf _snprintf
#endif
#include <stdlib.h>					// malloc(), free()
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
#include <AR/config.h>
#include <AR/video.h>
#include <AR/param.h>			// arParamDisp()
#include <AR/ar.h>
// #include <AR/gsub_lite.h>
#include <AR/gsub.h>
#include <AR/video.h>
#include "model_obj.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

// for using LCM
#include <lcm/lcm-cpp.hpp>
#include "comm_robot/comm_rob_t.hpp"

using namespace glm;
using namespace std;
// using namespace lcm;
// ============================================================================
//	Constants
// ============================================================================

#define             CPARA_NAME       "xzpdata2/test4.dat"// "xzpdata/xzpC930.dat"//"xzpdata/xzpC930.dat"//"408exp.dat"//"xzp_camera_para.dat"// "Data/c930.dat"// camera_para120.dat"     // camera calibration
#define             VPARA_NAME       "Data/cameraSetting-%08x%08x.dat"
//#define             PATT_NAME        "Data/hiro.patt"



// ============================================================================
//	Global variables
// ============================================================================

//// Preferences.
//static int windowed = TRUE;                     // Use windowed (TRUE) or fullscreen mode (FALSE) on launch.
//static int windowWidth = 640;					// Initial window width, also updated during program execution.
//static int windowHeight = 480;                  // Initial window height, also updated during program execution.
//static int windowDepth = 32;					// Fullscreen mode bit depth.
//static int windowRefresh = 0;					// Fullscreen mode refresh rate. Set to 0 to use default rate.


ARHandle           *arHandle;
ARPattHandle       *arPattHandle;
AR3DHandle         *ar3DHandle;
ARGViewportHandle  *vp;
int                 xsize, ysize;
int                 flipMode = 0;
// int                 patt_id;
double              patt_width = 80.0;
int                 countN = 0;
char                fps[256];
char                errValue[256];
char                errValue2[256];
int                 distF = 0;
int                 contF = 0;
ARParamLT          *gCparamLT = NULL;
int saveImage;
int curImageNumber = 0;

 // Path of the pattern, define 1 ref and 5 object pattern
// char const *Patterns[] = { "xzpdata2/Or_16.patt", "xzpdata2/A_16.patt", "xzpdata2/B_16.patt","xzpdata2/C_16.patt", "xzpdata2/D_16.patt", 
// "xzpdata2/E_16.patt", "xzpdata2/F_16.patt", "xzpdata2/G_16.patt", "xzpdata2/H_16.patt","xzpdata2/I_16.patt", "xzpdata2/J_16.patt", 
// "xzpdata2/K_16.patt", "xzpdata2/L_16.patt", "xzpdata2/M_16.patt", "xzpdata2/N_16.patt","xzpdata2/O_16.patt", "xzpdata2/P_16.patt", 
// "xzpdata2/Q_16.patt", "xzpdata2/R_16.patt","xzpdata2/S_16.patt", "xzpdata2/T_16.patt"};


char const *Patterns[] = { "xzpdata2/Or_16.patt", "xzpdata2/A_16.patt", "xzpdata2/B_16.patt","xzpdata2/C_16.patt", "xzpdata2/D_16.patt", 
"xzpdata2/E_16.patt", "xzpdata2/F_16.patt", "xzpdata2/G_16.patt", "xzpdata2/H_16.patt","xzpdata2/I_16.patt", "xzpdata2/J_16.patt", 
"xzpdata2/K_16.patt", "xzpdata2/L_16.patt", "xzpdata2/M_16.patt", "xzpdata2/N_16.patt","xzpdata2/O_16.patt", "xzpdata2/P_16.patt", 
"xzpdata2/Q_16.patt", "xzpdata2/R_16.patt","xzpdata2/S_16.patt", "xzpdata2/T_16.patt"};

//char const *Patterns[] = { "xzpdata2/test10.patt","xzpdata2/test11.patt"};
//char const *Patterns[] = { "xzpdata/Or_16.patt","xzpdata/A_16.patt","xzpdata/B_16.patt","xzpdata/C_16.patt","xzpdata/D_16.patt",
						// "xzpdata/E_16.patt","xzpdata/F_16.patt","xzpdata/G_16.patt","xzpdata/H_16.patt","xzpdata/I_16.patt"};//,"c.patt" };
// char const *Patterns[] = { "Data/R.patt","Data/A.patt", "Data/B.patt", "Data/C.patt", "Data/D.patt", "Data/E.patt" };
// char const *Patterns[] = { "M_Data/0.patt","M_Data/1.patt", "M_Data/2.patt", "M_Data/3.patt", "M_Data/4.patt"
				// , "M_Data/5.patt",

 // };
static int			number_Patterns = sizeof(Patterns) / sizeof(*Patterns);		// Calculate the number of patterns
int					*patt_id = new int[number_Patterns];


// Create array for Virtual objects/ number of objects you want to load and draw
model_obj image[6];


// create lcm object 
string udpm = "udpm://239.255.76.67:7667?ttl=1";
lcm::LCM lcm_handler(udpm);
// create a custom message handler 
comm_robot::comm_rob_t locmsg_handler;
// ============================================================================
//	Structure.
// ============================================================================

// Define a structure for the pattern
typedef struct {				
	const char *patt_name;			// has to be Patterns[number_of_the_pattern -1 ]
	int     patt_id;			// set as -1
	int     model_id;			// set as number_of_the_pattern
	int     visible;			// set as 0
	double  width;				// width of the pattern
	double  center[2];			// position of the center of the pattern, here { 0.0, 0.0 }
	double  trans[3][4];		// transition matrix, initially leave it blank
} OBJECT_T;


// ============================================================================
//	Function definition.
// ============================================================================

static void   init(int argc, char *argv[]);
static void   keyFunc( unsigned char key, int x, int y );
static void   cleanup(void);
static void   mainLoop(void);
static void   draw(ARdouble trans[3][4], double width, int numberFigure);
