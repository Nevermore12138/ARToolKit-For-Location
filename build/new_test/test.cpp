
#include "test.hpp"

OBJECT_T   object[21] = {
	{ Patterns[0], -1, 1, 0, 160.0, { 0.0, 0.0 } }, // 175//180是识别到的框大小
	{ Patterns[1], -1, 2, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[2], -1, 3, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[3], -1, 4, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[4], -1, 5, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[5], -1, 6, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[6], -1, 7, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[7], -1, 8, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[8], -1, 9, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[9], -1, 10, 0, 160.0, { 0.0, 0.0 } }, 
	{ Patterns[10], -1, 11, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[11], -1, 12, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[12], -1, 13, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[13], -1, 14, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[14], -1, 15, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[15], -1, 16, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[16], -1, 17, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[17], -1, 18, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[18], -1, 19, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[19], -1, 20, 0, 160.0, { 0.0, 0.0 } },
	{ Patterns[20], -1, 21, 0, 160.0, { 0.0, 0.0 } }
};

// main function

int main(int argc, char *argv[])
{
    // initialization of glut
    glutInit(&argc, argv);
    
    // initialization for the project
    init(argc, argv);


    argSetDispFunc( mainLoop, 1 );
	argSetKeyFunc( keyFunc );
	countN = 0;
    fps[0] = '\0';
	arUtilTimerReset();


	// Load the Virtual Objects
	image[0].Load("OBJ/rf.obj");
	image[1].Load("OBJ/number_one.obj");
	image[2].Load("OBJ/number_two.obj");
	image[3].Load("OBJ/number_three.obj");
	image[4].Load("OBJ/number_four.obj");
	image[5].Load("OBJ/number_five.obj");
    
    argMainLoop();
	return (0);
}

// key function 
static void keyFunc( unsigned char key, int x, int y )
{
    int   value = 0;

    switch (key) {
		case 0x1b:
        case 'q':
        case 'Q':
			cleanup();
			exit(0);
			break;
		case '1':
		case '-':
        	arGetLabelingThresh( arHandle, &value );
        	value -= 5;
        	if( value < 0 ) value = 0;
        	arSetLabelingThresh( arHandle, value );
        	ARLOG("thresh = %d\n", value);
        	break;
		case '2':
		case '+':
        	arGetLabelingThresh( arHandle, &value );
       		value += 5;
        	if( value > 255 ) value = 255;
        	arSetLabelingThresh( arHandle, value );
        	ARLOG("thresh = %d\n", value);
        	break;
		case 'd':
		case 'D':
        	arGetDebugMode( arHandle, &value );
       		value = 1 - value;
        	arSetDebugMode( arHandle, value );
            break;
        case 'h':
        case 'H':
            if( flipMode & AR_GL_FLIP_H ) flipMode = flipMode & AR_GL_FLIP_V;
            else                         flipMode = flipMode | AR_GL_FLIP_H;
            argViewportSetFlipMode( vp, flipMode );
            break;
        case 'v':
        case 'V':
            if( flipMode & AR_GL_FLIP_V ) flipMode = flipMode & AR_GL_FLIP_H;
            else                         flipMode = flipMode | AR_GL_FLIP_V;
            argViewportSetFlipMode( vp, flipMode );
        	break;
        case ' ':
            distF = 1 - distF;
            if( distF ) {
                argViewportSetDistortionMode( vp, AR_GL_DISTORTION_COMPENSATE_ENABLE );
            } else {
                argViewportSetDistortionMode( vp, AR_GL_DISTORTION_COMPENSATE_DISABLE );
            }
            break;
        case 'c':
            contF = 1 - contF;
            break;
		case '?':
		case '/':
			ARLOG("Keys:\n");
			ARLOG(" [esc]         Quit demo.\n");
			ARLOG(" - and +       Adjust threshhold.\n");
			ARLOG(" d             Activate / deactivate debug mode.\n");
			ARLOG(" h and v       Toggle horizontal / vertical flip mode.\n");
            ARLOG(" [space]       Toggle distortion compensation.\n");
			ARLOG(" ? or /        Show this help.\n");
			ARLOG("\nAdditionally, the ARVideo library supplied the following help text:\n");
			arVideoDispOption();
			break;
		default:
			break;
	}
}

static void mainLoop(void)
{

	// stringstream ss;

	static int ms_prev;
	int ms;
	float s_elapsed;

	static int imageNumber = 0;
	static int      contF2 = 0;						// Counter for the error calculation optimizer
	static ARUint8 *dataPtr = NULL;
	ARMarkerInfo   *markerInfo;
	int             markerNum;						// Found possible markers
	ARdouble        err;

	int             imageProcMode;
	int             debugMode;
	int             i, j, r, ii;
	double			wmat1[3][4], wmat2[3][4];		// Matrices for the changing of the reference/origin frame
	int				flag = 0;						// Flag for the matched references
	int				*k = new int[number_Patterns];  // Creates a dynamic array for k

	for (int i = 0; i < number_Patterns; i++){		// Sets all the entries from k to -1

		k[i] = -1;
	}

	int value;
	ARdouble        valuePrint;
	// String/Stream for sending the data to the socket
    // stringstream write_UDP;
	// string UDP_str;
	// locmsg_handler.timestamp = 0;
	memset(locmsg_handler.position, 0, sizeof(double)*3);
	memset(locmsg_handler.orientation, 0, sizeof(double)*3);
	// locmsg_handler.robot_id = 1;
	locmsg_handler.enabled = true;
	ms = glutGet(GLUT_ELAPSED_TIME);
	s_elapsed = (float)(ms - ms_prev) * 0.001f;
	ms_prev = ms;
	ARLOGe("Time[ms] %d \n", ms_prev);
	locmsg_handler.timestamp = ms_prev;

	/* grab a video frame */
	if ((dataPtr = (ARUint8 *)arVideoGetImage()) == NULL) {
		arUtilSleep(2);
		return;
	}



	argDrawMode2D(vp);
	arGetDebugMode(arHandle, &debugMode);
	if (debugMode == 0) {
		argDrawImage(dataPtr);
	}
	else {
		arGetImageProcMode(arHandle, &imageProcMode);
		if (imageProcMode == AR_IMAGE_PROC_FRAME_IMAGE) {
			argDrawImage(arHandle->labelInfo.bwImage);
		}
		else {
			argDrawImageHalf(arHandle->labelInfo.bwImage);
		}
	}
//argDrawImage(dataPtr);
	
	//image = (ARUint8 *)arVideoGetImage();
	char imageNumberText[15];
	if ((saveImage == 1) && (ms_prev >= 1000*curImageNumber)){
	sprintf(imageNumberText, "image-%04d.jpg", imageNumber++);
	}
	string folderName = "image";
		

	if (arDetectMarker(arHandle, dataPtr) < 0) {
		cleanup();
		exit(0);
	}

	if (countN % 60 == 0) {
		sprintf(fps, "%f[fps]", 60.0 / arUtilTimer());
		arUtilTimerReset();
	}
	countN++;
	
	
	glColor3f(0.0f, 1.0f, 0.0f);
	argDrawStringsByIdealPos(fps, 10, ysize - 30);

	arGetLabelingThresh(arHandle, &value);
	valuePrint = value;
	glColor3f(1.0f, 1.0f, 0.0f);
	sprintf(errValue2, "Threshold = %f", valuePrint);
	argDrawStringsByIdealPos(errValue2, 10, ysize - 60);
	

	// Get the number of possible marks/patterns in the video frame<从这里开始判别>
	markerNum = arGetMarkerNum(arHandle);
	// if there is no possible marks/patterns
	if (markerNum == 0) {

		// prints a 3x4 zeros-matrix and sends it to the socket
		for (i = 1; i <= number_Patterns - 1; i++){			// i begins at 1, since 0 would be for the reference/origin
			object[i].visible = 0;
			for (j = 0; j < 3; j++) {
				printf("NaN,");
				// write_UDP << "NaN" << ',';
				for (ii = 0; ii < 3; ii++) {
					printf("NaN,");
					// write_UDP << "NaN" << ',';
				}
				printf("\n");
				// write_UDP << '\n';
			}
			printf("******************************************************\n");
		}

		argSwapBuffers();
		return;
	}	

	/* check for object visibility */
	markerInfo = arGetMarker(arHandle);

	// Initiates the loop for tracking the markers if there is at least one possible marker found
	for (j = 0; j < markerNum; j++) {
		//Loop runs for the number of patterns
		for (i = 0; i <= number_Patterns - 1; i++) {
			//Compare which marker is corresponds to which pattern
			if (patt_id[i] == markerInfo[j].id) {	
				// Set flag to 1 if there is at least one marker found
				if (i == 0){
					flag = 1;			
				}
				// Check compatibility(互换性) and set the k to the number of the marker	
				if (k[i] == -1) {
					if (markerInfo[j].cf >= 0.7) 
						k[i] = j;
				}
				else if (markerInfo[j].cf > markerInfo[k[i]].cf) 
					k[i] = j;
			}
		}
	}
	
	// In case no match for the reference is found
	if (flag == 0) {
		// Set the visibility of the pattern to 0
		for (i = 1; i <= number_Patterns - 1; i++){		// Again it begins on i=1, since we dont want the reference/origin
			object[i].visible = 0;

			// Print all NaN matrices for all patterns
			for (j = 0; j < 3; j++) {
				printf("NaN,");
				// write_UDP << "NaN" << ',';
				for (ii = 0; ii < 3; ii++) {
					printf("NaN,");
					// write_UDP << "NaN" << ',';
				}
				printf("\n");
				// write_UDP << '\n';
			}
			printf("******************************************************\n");
		}
		contF2 = 0;
		argSwapBuffers();

		return;
	}


	// Get the transformation between the marker and the real camera <已确定有匹配的特征标识>
	for (i = 0; i <= number_Patterns - 1; i++){
		if (k[i] != -1){

			// If the object is visible use arGetTransMatSquareCont for a better localization
			if (object[i].visible) {
				err = arGetTransMatSquareCont(ar3DHandle, &(markerInfo[k[i]]), object[i].trans, object[i].width, object[i].trans);
			}
			// If the object is not visible use arGetTransMatSquare for the localization
			else {
				err = arGetTransMatSquare(ar3DHandle, &(markerInfo[k[i]]), object[i].width, object[i].trans);
			}
			
			
			if (i != 0)
			{
				object[i].visible = 0;
				
			}
			else
			{ 
				object[0].visible = 1;
			}

			
			draw(object[i].trans, object[i].width, i);
			
				
		}
	}
	
	// Set the first pattern as the origin/reference frame
	arUtilMatInv(object[0].trans, wmat1);

	// Gets the distance between each robot and the origin/reference frame
	for (r = 1; r < number_Patterns; r++) {
		// if the robot is found
		if (k[r] != -1) {
			// gets the transformation between robot and origin
			arUtilMatMul(wmat1, object[r].trans, wmat2);
			// prints the transformation of the robots
			locmsg_handler.robot_id=r;
			
			for (j = 0; j < 3; j++) {
				// send position information (初始位置是wmat2[0][0])
				locmsg_handler.position[j] = wmat2[j][3];

				for (i = 0; i < 4; i++) {
					printf("%8.4f ", wmat2[j][i]);
					// write_UDP << wmat2[j][i] << ',';
				}
				printf("\n");
				// write_UDP << '\n';
			}
			locmsg_handler.orientation[2]=-atan2(wmat2[0][1],wmat2[0][0]);
			lcm_handler.publish("XZPtest", &locmsg_handler);
			
			printf("******************************************************\n");

		}
		// if the robot is not found
		// print a '-1' matrix for the transformation matrix
		else {
			for (j = 0; j < 3; j++) {
				for (i = 0; i < 4; i++) {
					printf("NaN,");
					// write_UDP << "NaN" << ',';
				}
				printf("\n");
				// write_UDP << '\n';
			}
			printf("******************************************************\n");
		}
	}



		contF2 = 1;
		argSwapBuffers();
}

static void   init(int argc, char *argv[])
{
    ARParam         cparam;
    ARGViewport     viewport;
    char            vconf[512];
    AR_PIXEL_FORMAT pixFormat;
    ARUint32        id0, id1;
    int             i, r;
    
    if(!lcm_handler.good()) {
    	printf("lcm is not good");
    	exit(1);
    }
    if( argc == 1 ) 
    	vconf[0] = '\0';
    else 
    {
        strcpy( vconf, argv[1] );
        for( i = 2; i < argc; i++ ) {strcat(vconf, " "); strcat(vconf,argv[i]);}
    }
	char abc[] = "-dev=/dev/video0 -width=1920 -height=1080"; //set the camera 1 as image device usually 0 is onboard camera and 1 is the first external camera 
	printf("I will print the vconf %s\n", vconf);
    /* open the video path */
	ARLOGi("Using video configuration '%s'.\n", vconf);
    if( arVideoOpen( abc ) < 0 ) 
    	exit(0);  // vconf
    if( arVideoGetSize(&xsize, &ysize) < 0 ) 
    	exit(0);
    ARLOGi("Image size (x,y) = (%d,%d)\n", xsize, ysize);
    if( (pixFormat=arVideoGetPixelFormat()) < 0 ) 
    	exit(0);
    if( arVideoGetId( &id0, &id1 ) == 0 ) {
        ARLOGi("Camera ID = (%08x, %08x)\n", id1, id0);
        sprintf(vconf, VPARA_NAME, id1, id0);
        if( arVideoLoadParam(vconf) < 0 ) {  // only about 1394
            ARLOGe("No camera setting data!!\n");
        }
    }

    /* set the initial camera parameters */
    if( arParamLoad(CPARA_NAME, 1, &cparam) < 0 ) {
        ARLOGe("Camera parameter load error !!\n");
        exit(0);
    }
    // change the size of the camera
    xsize = 1920;
    ysize = 1080;
    arParamChangeSize( &cparam, xsize, ysize, &cparam );
    ARLOG("*** Camera Parameter ***\n");
    arParamDisp( &cparam );
    if ((gCparamLT = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
        ARLOGe("Error: arParamLTCreate.\n");
        exit(-1);
    }
    
    if( (arHandle=arCreateHandle(gCparamLT)) == NULL ) {
        ARLOGe("Error: arCreateHandle.\n");
        exit(0);
    }
    if( arSetPixelFormat(arHandle, pixFormat) < 0 ) {
        ARLOGe("Error: arSetPixelFormat.\n");
        exit(0);
    }

    if (arSetDebugMode(arHandle, AR_DEBUG_DISABLE) < 0) {
        ARLOGe("setupCamera(): Error: arSetDebugMode.\n");
    }

    if( (ar3DHandle=ar3DCreateHandle(&cparam)) == NULL ) {
        ARLOGe("Error: ar3DCreateHandle.\n");
        exit(0);
    }

    if( (arPattHandle=arPattCreateHandle()) == NULL ) {
        ARLOGe("Error: arPattCreateHandle.\n");
        exit(0);
    }
    // Loop that sets each marker to a patt_id
	for (r = 0; r < number_Patterns; r++) {
		if ((patt_id[r] = arPattLoad(arPattHandle, Patterns[r])) < 0) {
			ARLOGe("pattern load error !!\n");
			exit(0);
		}
	}
    arPattAttach( arHandle, arPattHandle );

    /* open the graphics window */
    viewport.sx = 0;
    viewport.sy = 0;
    viewport.xsize = xsize;
    viewport.ysize = ysize;
    if( (vp=argCreateViewport(&viewport)) == NULL ) exit(0);
    argViewportSetCparam( vp, &cparam );
    argViewportSetPixFormat( vp, pixFormat );
//    argViewportSetDispMethod( vp, AR_GL_DISP_METHOD_GL_DRAW_PIXELS );
    argViewportSetDistortionMode( vp, AR_GL_DISTORTION_COMPENSATE_DISABLE );

	if (arVideoCapStart() != 0) {
        ARLOGe("video capture start error !!\n");
        exit(0);
	}
}

/* cleanup function called when program exits */
static void cleanup(void)
{
    arVideoCapStop();
    argCleanup();
	arPattDetach(arHandle);
	arPattDeleteHandle(arPattHandle);
	ar3DDeleteHandle(&ar3DHandle);
	arDeleteHandle(arHandle);
    arParamLTFree(&gCparamLT);
    arVideoClose();
}

static void draw(ARdouble trans[3][4], double  width, int i)
{

	// Drawing parameters
	ARdouble  gl_para[16];
	GLfloat   mat_diffuse[] = { 1.0f, 1.0f, 0.0f, 0.0f };	// Color
	GLfloat   mat_flash[] = { 1.0f, 1.0f, 1.0f, 0.0f };
	GLfloat   mat_flash_shiny[] = { 50.0f };
	GLfloat   light_position[] = { 100.0f, -200.0f, 200.0f, 0.0f };
	GLfloat   light_ambi[] = { 0.1f, 0.1f, 0.1f, 0.0f };
	GLfloat   light_color[] = { 1.0f, 1.0f, 0.0f, 0.0f };

	argDrawMode3D(vp);
	glClearDepth(1.0);
	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	/* load the camera transformation matrix */
	argConvGlpara(trans, gl_para);
	glMatrixMode(GL_MODELVIEW);
#ifdef ARDOUBLE_IS_FLOAT
	glLoadMatrixf(gl_para);
#else
	glLoadMatrixd(gl_para);
#endif

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambi);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_color);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_color);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_flash);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_flash_shiny);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_diffuse);

#if 1
	// https://www.opengl.org/resources/libraries/glut/spec3/spec3.html : the library explaination




	// Parameters of the drawing
	//glPushMatrix();
	glTranslatef(0.0f, 0.0f, 0.0f);		// Set the translation of the drawing
	glRotatef(90, 1, 0, 0);				// Set the rotation of the drawing
	glScalef(width, width, width);		// Set the scaling of the drawing

	// Draw the virtual object
	// image[i].Draw();


	
#else
	glTranslatef(0.0f, 0.0f, 20.0f);
	glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
	glutSolidTeapot(40.0);
#endif
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	glDisable(GL_DEPTH_TEST);
	
}
