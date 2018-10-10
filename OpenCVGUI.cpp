// Prevent Visual Studio Intellisense from defining _WIN32 and _MSC_VER when we use 
// Visual Studio to edit Linux or Borland C++ code.
#ifdef __linux__
#	undef _WIN32
#endif // __linux__
#if defined(__GNUC__) || defined(__BORLANDC__)
#	undef _MSC_VER
#endif // defined(__GNUC__) || defined(__BORLANDC__)

#include "OpenCVGUI.h"

THREAD_PROC_RETURN_VALUE OpenCVGUIThread(void* pParam)
{
	int videoid = (intptr_t)pParam;
	int c = 0, i = 0, offset = 0;
	double angle = 0, d0 = 0, d1 = 0, d2 = 0;
	int days = 0, hours = 0, minutes = 0, seconds = 0;
	double deccsec = 0;
	BOOL bOSD = TRUE;
	BOOL bOrientationCircle = TRUE;
	BOOL bDispLLA = TRUE;
	BOOL bDispYPR = TRUE;
	BOOL bDispAltitudeAGL = FALSE;
	BOOL bDispDVL = TRUE;
	BOOL bDispGPS = TRUE;
	BOOL bDispSOG = FALSE;
	BOOL bDispEPU = FALSE;
	BOOL bMap = TRUE;
	BOOL bFullMap = FALSE;
	BOOL bRotatingMap = FALSE;
	BOOL bShowSonar = FALSE;
	BOOL bShowOtherOverlays = TRUE;
	COORDSYSTEM2IMG csMap2FullImg;
	BOOL bVideoRecording = FALSE;
	BOOL bDispRecordSymbol = FALSE;
	BOOL bDispPlaySymbol = FALSE;
	BOOL bDispPauseSymbol = FALSE;
	BOOL bEnableAltRCMode = FALSE;
	BOOL bZQSDPressed = FALSE;
	BOOL bExtendedMenu = FALSE;
	BOOL bColorsExtendedMenu = FALSE;
	BOOL bCISCREAOSDExtendedMenu = FALSE;
	BOOL bOtherOSDOptionsExtendedMenu = FALSE;
	CvPoint PlaySymbolPoints[3];
	int nbPlaySymbolPoints = sizeof(PlaySymbolPoints);
	CvPoint PauseSymbolPoints[4];
	int nbPauseSymbolPoints = sizeof(PauseSymbolPoints);
	char strtime_snap[MAX_BUF_LEN];
	char snapfilename[MAX_BUF_LEN];
	char picsnapfilename[MAX_BUF_LEN];
	char kmlsnapfilename[MAX_BUF_LEN];
	FILE* kmlsnapfile = NULL;
	char s = 0;
	char szText[MAX_BUF_LEN];
	char windowname[MAX_BUF_LEN];
	int colortextid = 0;
	int colorsonarlidarid = 0;
	int colormapid = 0;
	CvScalar colortext;
	CvScalar colormap;
	CvFont font;
	CHRONO chrono_recording;
	CHRONO chrono_playing;
	CHRONO chrono_pausing;
	CHRONO chrono_alarms;
	CHRONO chrono_epu;

	switch (robid)
	{
	case BUBBLE_ROBID:
	case ETAS_WHEEL_ROBID:
		bEnableAltRCMode = TRUE;
		break;
	case MOTORBOAT_SIMULATOR_ROBID:
	case MOTORBOAT_ROBID:
	case BUGGY_SIMULATOR_ROBID:
	case BUGGY_ROBID:
		bEnableAltRCMode = TRUE;
		break;
	default:
		bEnableAltRCMode = FALSE;
		break;
	}

	if (bDisablePathfinderDVL&&bDisableNortekDVL) bDispDVL = FALSE; else bDispDVL = TRUE;

	memset(szText, 0, sizeof(szText));
	memset(windowname, 0, sizeof(windowname));

	StartChrono(&chrono_recording);
	StartChrono(&chrono_playing);
	StartChrono(&chrono_pausing);
	StartChrono(&chrono_alarms);
	StartChrono(&chrono_epu);

	// Sometimes needed on Linux to get windows-related functions working properly in multiple threads, sometimes not...
	//EnterCriticalSection(&OpenCVCS);
#ifndef USE_OPENCV_HIGHGUI_CPP_API
	//cvStartWindowThread();
#else
	//cv::startWindowThread();
#endif // !USE_OPENCV_HIGHGUI_CPP_API
	//LeaveCriticalSection(&OpenCVCS);

	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0.0, 1, 8);
	colortextid = 0;
	colortext = CV_RGB(0, 255, 128);
	colorsonarlidarid = 0;
	colormapid = 0;
	colormap = CV_RGB(0, 255, 0);

	for (;;)
	{
		if (windowname[0] != 0)
		{
#ifdef ENABLE_OPENCV_HIGHGUI_THREADS_WORKAROUND
			EnterCriticalSection(&OpenCVCS);
#ifndef USE_OPENCV_HIGHGUI_CPP_API
			c = cvWaitKey(captureperiod);
#else
			c = cv::waitKey(captureperiod);
#endif // !USE_OPENCV_HIGHGUI_CPP_API
			LeaveCriticalSection(&OpenCVCS);
#else
#ifndef USE_OPENCV_HIGHGUI_CPP_API
			c = cvWaitKey(captureperiod);
#else
			c = cv::waitKey(captureperiod);
#endif // !USE_OPENCV_HIGHGUI_CPP_API
#endif // ENABLE_OPENCV_HIGHGUI_THREADS_WORKAROUND
	}
		else mSleep(captureperiod);

		if (bExit) break;
#pragma region IMAGES
		if (bEnableOpenCVGUIs[videoid])
		{
			if (windowname[0] == 0)
			{
				sprintf(windowname, "UxVOpenCVGUI%d", videoid);
				EnterCriticalSection(&OpenCVCS);
#ifndef USE_OPENCV_HIGHGUI_CPP_API
				cvNamedWindow(windowname, CV_WINDOW_AUTOSIZE);
				cvMoveWindow(windowname, videoimgwidth*videoid, 0);
#else
				cv::namedWindow(windowname, cv::WINDOW_AUTOSIZE);
				cv::moveWindow(windowname, videoimgwidth*videoid, 0);
#endif // !USE_OPENCV_HIGHGUI_CPP_API
				LeaveCriticalSection(&OpenCVCS);
			}
		}
		else
		{
			if (windowname[0] != 0)
			{
				if (bVideoRecording)
				{
					EnterCriticalSection(&VideoRecordRequestsCS[videoid]);
					VideoRecordRequests[videoid]--;
					LeaveCriticalSection(&VideoRecordRequestsCS[videoid]);
					bVideoRecording = FALSE;
					bDispRecordSymbol = FALSE;
				}
				if (bMissionRunning)
				{
					//AbortMission();
					bDispPlaySymbol = FALSE;
				}
				EnterCriticalSection(&OpenCVCS);
#ifndef USE_OPENCV_HIGHGUI_CPP_API
				cvDestroyWindow(windowname);
				cvNamedWindow(windowname, CV_WINDOW_AUTOSIZE);
				cvDestroyWindow(windowname);
#else
				cv::destroyWindow(windowname);
				cv::namedWindow(windowname, cv::WINDOW_AUTOSIZE);
				cv::destroyWindow(windowname);
#endif // !USE_OPENCV_HIGHGUI_CPP_API
				LeaveCriticalSection(&OpenCVCS);
				memset(windowname, 0, sizeof(windowname));
			}
			mSleep(100);
			continue;
		}

		EnterCriticalSection(&dispimgsCS[videoid]);
		if (bShowVideoOpenCVGUIs[videoid])
		{
			EnterCriticalSection(&imgsCS[videoid]);
			cvCopy(imgs[videoid], dispimgs[videoid], 0);
			LeaveCriticalSection(&imgsCS[videoid]);
		}
		else
		{
			cvSet(dispimgs[videoid], CV_RGB(0, 0, 0), NULL);
		}
		if (bShowSonar)
		{
			EnterCriticalSection(&SeanetOverlayImgCS);
			CopyOverlay(SeanetOverlayImg, dispimgs[videoid]);
			LeaveCriticalSection(&SeanetOverlayImgCS);
		}
		if (bShowOtherOverlays)
		{
			//if (bDynamicSonarLocalization) 		
			//{
			//	EnterCriticalSection(&DynamicSonarLocalizationOverlayImgCS);
			//	CopyOverlay(DynamicSonarLocalizationOverlayImg, dispimgs[videoid]);
			//	LeaveCriticalSection(&DynamicSonarLocalizationOverlayImgCS);
			//}
			if (bExternalVisualLocalization&&(videoid == videoid_externalvisuallocalization))
			{
				EnterCriticalSection(&ExternalVisualLocalizationOverlayImgCS);
				CopyOverlay(ExternalVisualLocalizationOverlayImg, dispimgs[videoid]);
				LeaveCriticalSection(&ExternalVisualLocalizationOverlayImgCS);
			}
			if ((bWallDetection||bWallTrackingControl||bWallAvoidanceControl))
			{
				EnterCriticalSection(&WallOverlayImgCS);
				CopyOverlay(WallOverlayImg, dispimgs[videoid]);
				LeaveCriticalSection(&WallOverlayImgCS);
			}
			if ((bPipelineDetection||bPipelineTrackingControl)&&(videoid == videoid_pipeline))
			{
				EnterCriticalSection(&PipelineOverlayImgCS);
				CopyOverlay(PipelineOverlayImg, dispimgs[videoid]);
				LeaveCriticalSection(&PipelineOverlayImgCS);
			}
			if ((bBallDetection||bBallTrackingControl)&&(videoid == videoid_ball))
			{
				EnterCriticalSection(&BallOverlayImgCS);
				CopyOverlay(BallOverlayImg, dispimgs[videoid]);
				LeaveCriticalSection(&BallOverlayImgCS);
			}
			if ((bVisualObstacleDetection||bVisualObstacleAvoidanceControl)&&(videoid == videoid_visualobstacle))
			{
				EnterCriticalSection(&VisualObstacleOverlayImgCS);
				CopyOverlay(VisualObstacleOverlayImg, dispimgs[videoid]);
				LeaveCriticalSection(&VisualObstacleOverlayImgCS);
			}
			if ((bSurfaceVisualObstacleDetection||bSurfaceVisualObstacleAvoidanceControl)&&(videoid == videoid_surfacevisualobstacle))
			{
				EnterCriticalSection(&SurfaceVisualObstacleOverlayImgCS);
				CopyOverlay(SurfaceVisualObstacleOverlayImg, dispimgs[videoid]);
				LeaveCriticalSection(&SurfaceVisualObstacleOverlayImgCS);
			}
			if ((bPingerDetection||bPingerTrackingControl)&&(videoid == videoid_pinger))
			{
				EnterCriticalSection(&PingerOverlayImgCS);
				CopyOverlay(PingerOverlayImg, dispimgs[videoid]);
				LeaveCriticalSection(&PingerOverlayImgCS);
			}
			if ((bMissingWorkerDetection||bMissingWorkerTrackingControl)&&(videoid == videoid_missingworker))
			{
				EnterCriticalSection(&MissingWorkerOverlayImgCS);
				CopyOverlay(MissingWorkerOverlayImg, dispimgs[videoid]);
				LeaveCriticalSection(&MissingWorkerOverlayImgCS);
			}
		}
		LeaveCriticalSection(&dispimgsCS[videoid]);
#pragma endregion

		EnterCriticalSection(&StateVariablesCS);
#pragma region KEYS
		switch ((char)TranslateKeys(c))
		{
		case 'z':
			u += 0.1*u_max;
			u = (u > u_max)? u_max: u;
			switch (robid)
			{
			case SAILBOAT_SIMULATOR_ROBID:
			case VAIMOS_ROBID:
			case SAILBOAT_ROBID:
			case SAILBOAT2_ROBID:
				break;
			case MOTORBOAT_SIMULATOR_ROBID:
			case MOTORBOAT_ROBID:
			case BUGGY_SIMULATOR_ROBID:
			case BUGGY_ROBID:
				if (bEnableAltRCMode) u = u_max;
				break;
			case BUBBLE_ROBID:
			case ETAS_WHEEL_ROBID:
				if (bEnableAltRCMode) u = u_max;
				if (!bHeadingControl) uw = 0;
				break;
			default:
				if (!bHeadingControl) uw = 0;
				break;
			}
			bZQSDPressed = TRUE;
			break;
		case 's':
			u -= 0.1*u_max;
			u = (u < -u_max)? -u_max: u;
			switch (robid)
			{
			case SAILBOAT_SIMULATOR_ROBID:
			case VAIMOS_ROBID:
			case SAILBOAT_ROBID:
			case SAILBOAT2_ROBID:
				break;
			case MOTORBOAT_SIMULATOR_ROBID:
			case MOTORBOAT_ROBID:
			case BUGGY_SIMULATOR_ROBID:
			case BUGGY_ROBID:
				if (bEnableAltRCMode) u = -u_max;
				break;
			case BUBBLE_ROBID:
			case ETAS_WHEEL_ROBID:
				if (bEnableAltRCMode) u = -u_max;
				if (!bHeadingControl) uw = 0;
				break;
			default:
				if (!bHeadingControl) uw = 0;
				break;
			}
			bZQSDPressed = TRUE;
			break;
		case 'q':
			if (bHeadingControl)
			{
				wpsi += 0.03;
				wpsi = fmod_2PI(wpsi);
			}
			else
			{
				uw += 0.1*uw_max;
				uw = (uw > uw_max)? uw_max: uw;
				switch (robid)
				{
				case BUBBLE_ROBID:
				case ETAS_WHEEL_ROBID:
					if (bEnableAltRCMode) uw = uw_max;
					break;
				default:
					break;
				}
			}
			bZQSDPressed = TRUE;
			break;
		case 'd':
			if (bHeadingControl)
			{
				wpsi -= 0.03;
				wpsi = fmod_2PI(wpsi);
			}
			else
			{
				uw -= 0.1*uw_max;
				uw = (uw < -uw_max)? -uw_max: uw;
				switch (robid)
				{
				case BUBBLE_ROBID:
				case ETAS_WHEEL_ROBID:
					if (bEnableAltRCMode) uw = -uw_max;
					break;
				default:
					break;
				}
			}
			bZQSDPressed = TRUE;
			break;
		case 'f':
			switch (robid)
			{
			case MOTORBOAT_SIMULATOR_ROBID:
			case MOTORBOAT_ROBID:
			case BUGGY_SIMULATOR_ROBID:
			case BUGGY_ROBID:
			case BUBBLE_ROBID:
			case ETAS_WHEEL_ROBID:
				u_max += 0.025;
				u_max = (u_max > 1)? 1: u_max;
				break;
			default:
				if (bDepthControl)
				{
					wz += 0.1;
				}
				else if (bAltitudeAGLControl)
				{
					wagl += 0.1;
				}
				else
				{
					uv += 0.1*u_max_z;
					uv = (uv > u_max_z)? u_max_z: uv;
				}
				break;
			}
			break;
		case 'v':
			switch (robid)
			{
			case MOTORBOAT_SIMULATOR_ROBID:
			case MOTORBOAT_ROBID:
			case BUGGY_SIMULATOR_ROBID:
			case BUGGY_ROBID:
			case BUBBLE_ROBID:
			case ETAS_WHEEL_ROBID:
				u_max -= 0.025;
				u_max = (u_max < 0)? 0: u_max;
				break;
			default:
				if (bDepthControl)
				{
					wz -= 0.1;
				}
				else if (bAltitudeAGLControl)
				{
					wagl -= 0.1;
				}
				else
				{
					uv += 0.1*u_min_z;
					uv = (uv < u_min_z)? u_min_z: uv;
				}
				break;
			}
			break;
		case 'a':
			switch (robid)
			{
			case MOTORBOAT_SIMULATOR_ROBID:
			case MOTORBOAT_ROBID:
			case SAILBOAT_SIMULATOR_ROBID:
			case VAIMOS_ROBID:
			case SAILBOAT_ROBID:
			case SAILBOAT2_ROBID:
			case BUGGY_SIMULATOR_ROBID:
			case BUGGY_ROBID:
				if (!bHeadingControl) uw = 0;
				break;
			case QUADRO_SIMULATOR_ROBID:
			case COPTER_ROBID:
			case BLUEROV_ROBID:
			case ARDUCOPTER_ROBID:
				ul += 0.1;
				ul = (ul > 1)? 1: ul;
				break;
			case BUBBLE_ROBID:
			case TANK_SIMULATOR_ROBID:
			case ETAS_WHEEL_ROBID:
				uw_max += 0.1;
				uw_max = (uw_max > 1)? 1: uw_max;
				break;
			default:
				break;
			}
			break;
		case 'e':
			switch (robid)
			{
			case MOTORBOAT_SIMULATOR_ROBID:
			case MOTORBOAT_ROBID:
			case SAILBOAT_SIMULATOR_ROBID:
			case VAIMOS_ROBID:
			case SAILBOAT_ROBID:
			case SAILBOAT2_ROBID:
			case BUGGY_SIMULATOR_ROBID:
			case BUGGY_ROBID:
				if (!bHeadingControl) uw = 0;
				break;
			case QUADRO_SIMULATOR_ROBID:
			case COPTER_ROBID:
			case BLUEROV_ROBID:
			case ARDUCOPTER_ROBID:
				ul -= 0.1;
				ul = (ul < -1)? -1: ul;
				break;
			case BUBBLE_ROBID:
			case TANK_SIMULATOR_ROBID:
			case ETAS_WHEEL_ROBID:
				uw_max -= 0.1;
				uw_max = (uw_max < 0)? 0: uw_max;
				break;
			default:
				break;
			}
			break;
		case 'w':
			bBrakeControl = TRUE;
			bDistanceControl = FALSE;
			u = 0;
			break;
		case ' ':
			DisableAllHorizontalControls(); // wpsi = 0;
			break;
		case 'g':
			DisableAllControls(); // wpsi = 0; wz = 0;
			break;
		case 't':
			if (!bHeadingControl)
			{
				bHeadingControl = TRUE;
				wpsi = Center(psihat);
			}
			else
			{
				bHeadingControl = FALSE;
			}
			break;
		case 'y':
			if (!bDepthControl)
			{
				bDepthControl = TRUE;
				wz = Center(zhat);
			}
			else
			{
				bDepthControl = FALSE;
			}
			break;
		case 'Y':
			if (!bAltitudeAGLControl)
			{
				bAltitudeAGLControl = TRUE;
				wagl = altitude_AGL;
			}
			else
			{
				bAltitudeAGLControl = FALSE;
			}
			break;
		case 'U':
			if (!bPitchControl)
			{
				bPitchControl = TRUE;
			}
			else
			{
				bPitchControl = FALSE;
			}
			//if (bPitchControl) printf("Pitch control enabled.\n");
			//else printf("Pitch control disabled.\n");
			break;
		case 'H':
			if (!bRollControl)
			{
				bRollControl = TRUE;
			}
			else
			{
				bRollControl = FALSE;
			}
			//if (bRollControl) printf("Roll control enabled.\n");
			//else printf("Roll control disabled.\n");
			break;
		case 'b':
			lights += 0.1;
			lights = (lights > 1)? 1: lights;
			if (robid == BLUEROV_ROBID) joystick_buttons |= (1<<14);
			break;
		case 'n':
			lights -= 0.1;
			lights = (lights < 0)? 0: lights;
			if (robid == BLUEROV_ROBID) joystick_buttons |= (1<<13);
			break;
		case 'u':
			cameratilt += 0.1;
			cameratilt = (cameratilt > 1)? 1: cameratilt;
			if (robid == BLUEROV_ROBID) joystick_buttons |= (1<<10);
			break;
		case 'j':
			cameratilt -= 0.1;
			cameratilt = (cameratilt < -1)? -1: cameratilt;
			if (robid == BLUEROV_ROBID) joystick_buttons |= (1<<9);
			break;
		case 'N': 
			cameratilt = 0; 
			if (robid == BLUEROV_ROBID) joystick_buttons |= (1<<7);
			break;
		case 'o': bOSD = !bOSD; break;
		case 'c': bOrientationCircle = !bOrientationCircle; break;
		case 'm':
			bFullMap = FALSE;
			bMap = !bMap;
			break;
		case 'M':
			bMap = FALSE;
			bFullMap = !bFullMap;
			break;
		case '*': bRotatingMap = !bRotatingMap; break;
		case 'i': bShowVideoOpenCVGUIs[videoid] = !bShowVideoOpenCVGUIs[videoid]; break;
		case '$': 

			// Should cycle between different display modes...

			bShowSonar = !bShowSonar; 
			break;
		case ';': bShowOtherOverlays = !bShowOtherOverlays; break;
		case '+':
			if ((fabs(csMap.xMin) > 0.1)&&(fabs(csMap.xMax) > 0.1)&&(fabs(csMap.yMin) > 0.1)&&(fabs(csMap.yMax) > 0.1))
			{
				csMap.xMin *= 0.9; csMap.xMax *= 0.9; csMap.yMin *= 0.9; csMap.yMax *= 0.9;
			}
			break;
		case '-':
			if ((fabs(csMap.xMin) < 10000000)&&(fabs(csMap.xMax) < 10000000)&&(fabs(csMap.yMin) < 10000000)&&(fabs(csMap.yMax) < 10000000))
			{
				csMap.xMin /= 0.9; csMap.xMax /= 0.9; csMap.yMin /= 0.9; csMap.yMax /= 0.9;
			}
			break;

			// Disabled because casted to char, because problems on Linux if int, especially when using SHIFT...

#ifdef TEST_OPENCVGUI_ARROWS
		case 2490368: // Up arrow.
			if ((fabs(csMap.xMin) < 10000000)&&(fabs(csMap.xMax) < 10000000)&&(fabs(csMap.yMin) < 10000000)&&(fabs(csMap.yMax) < 10000000))
			{
				csMap.yMin += 1; csMap.yMax += 1;
			}
			break;
		case 2621440: // Down arrow.
			if ((fabs(csMap.xMin) < 10000000)&&(fabs(csMap.xMax) < 10000000)&&(fabs(csMap.yMin) < 10000000)&&(fabs(csMap.yMax) < 10000000))
			{
				csMap.yMin -= 1; csMap.yMax -= 1;
			}
			break;
		case 2424832: // Left arrow.
			if ((fabs(csMap.xMin) < 10000000)&&(fabs(csMap.xMax) < 10000000)&&(fabs(csMap.yMin) < 10000000)&&(fabs(csMap.yMax) < 10000000))
			{
				csMap.xMin -= 1; csMap.xMax -= 1;
			}
			break;
		case 2555904: // Right arrow.
			if ((fabs(csMap.xMin) < 10000000)&&(fabs(csMap.xMax) < 10000000)&&(fabs(csMap.yMin) < 10000000)&&(fabs(csMap.yMax) < 10000000))
			{
				csMap.xMin += 1; csMap.xMax += 1;
			}
			break;
#endif // TEST_OPENCVGUI_ARROWS
		case 'O':
			// gpssetenvcoordposition
			if (bCheckGNSSOK())
			{
				double latitude = 0, longitude = 0, altitude = 0;

				// We do not use GPS altitude for that as it is not reliable...
				// Assume that latitude,longitude is only updated by GPS...
				EnvCoordSystem2GPS(lat_env, long_env, alt_env, angle_env, Center(xhat), Center(yhat), Center(zhat), &latitude, &longitude, &altitude);
				lat_env = latitude; long_env = longitude;
			}
			break;
		case 'G':
			// gpslocalization
			if (bCheckGNSSOK())
			{
				// Should add speed...?
				// Should add altitude with a big error...?
				xhat = xhat & x_gps;
				yhat = yhat & y_gps;
				if (xhat.isEmpty || yhat.isEmpty)
				{
					xhat = x_gps;
					yhat = y_gps;
				}
			}
			break;
		case 'J':
			// enableautogpslocalization/disableautogpslocalization
			bGPSLocalization = !bGPSLocalization;
			break;
		case 'V':
			// enableautodvllocalization/disableautodvllocalization
			bDVLLocalization = !bDVLLocalization;
			break;
		case 'Z':
			// (re)setstateestimation
			xhat = interval(-MAX_UNCERTAINTY, MAX_UNCERTAINTY);
			yhat = interval(-MAX_UNCERTAINTY, MAX_UNCERTAINTY);
			zhat = interval(-MAX_UNCERTAINTY, MAX_UNCERTAINTY);
			psihat = interval(-MAX_UNCERTAINTY, MAX_UNCERTAINTY);
			vrxhat = interval(-MAX_UNCERTAINTY, MAX_UNCERTAINTY);
			omegazhat = interval(-MAX_UNCERTAINTY, MAX_UNCERTAINTY);
			break;
		case 'S':
			// staticsonarlocalization
			bStaticSonarLocalization = TRUE;
			break;
		case 'D':
			// enabledynamicsonarlocalization/disabledynamicsonarlocalization
			bDynamicSonarLocalization = !bDynamicSonarLocalization;
			break;
		case 'P':
			memset(strtime_snap, 0, sizeof(strtime_snap));
			EnterCriticalSection(&strtimeCS);
			strcpy(strtime_snap, strtime_fns());
			LeaveCriticalSection(&strtimeCS);
			for (i = 0; i < nbvideo; i++)
			{
				sprintf(snapfilename, "snap%d_%.64s.png", i, strtime_snap);
				sprintf(picsnapfilename, PIC_FOLDER"snap%d_%.64s.png", i, strtime_snap);
				EnterCriticalSection(&imgsCS[i]);
				if (!cvSaveImage(picsnapfilename, imgs[i], 0))
				{
					printf("Error saving a snapshot file.\n");
				}
				LeaveCriticalSection(&imgsCS[i]);
				EnvCoordSystem2GPS(lat_env, long_env, alt_env, angle_env, Center(xhat), Center(yhat), Center(zhat), &d0, &d1, &d2);
				sprintf(kmlsnapfilename, PIC_FOLDER"snap%d_%.64s.kml", i, strtime_snap);
				kmlsnapfile = fopen(kmlsnapfilename, "w");
				if (kmlsnapfile == NULL)
				{
					printf("Error saving a snapshot file.\n");
					continue;
				}
				fprintf(kmlsnapfile, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
				fprintf(kmlsnapfile, "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">\n");
				fprintf(kmlsnapfile, "<Document>\n<name>snap%d_%.64s</name>\n", i, strtime_snap);
				fprintf(kmlsnapfile, "\t<PhotoOverlay>\n\t\t<name>snap%d_%.64s</name>\n", i, strtime_snap);
				fprintf(kmlsnapfile, "\t\t<Camera>\n\t\t\t<longitude>%.8f</longitude>\n\t\t\t<latitude>%.8f</latitude>\n\t\t\t<altitude>%.3f</altitude>\n", d1, d0, d2);
				fprintf(kmlsnapfile, "\t\t\t<heading>%f</heading>\n\t\t\t<tilt>%f</tilt>\n\t\t\t<roll>%f</roll>\n", (fmod_2PI(-angle_env-Center(psihat)+3.0*M_PI/2.0)+M_PI)*180.0/M_PI, 0.0, 0.0);
				fprintf(kmlsnapfile, "\t\t\t<altitudeMode>relativeToGround</altitudeMode>\n\t\t\t<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode>\n\t\t</Camera>\n");
				fprintf(kmlsnapfile, "\t\t<Style>\n\t\t\t<IconStyle>\n\t\t\t\t<Icon>\n\t\t\t\t\t<href>:/camera_mode.png</href>\n\t\t\t\t</Icon>\n\t\t\t</IconStyle>\n");
				fprintf(kmlsnapfile, "\t\t\t<ListStyle>\n\t\t\t\t<listItemType>check</listItemType>\n\t\t\t\t<ItemIcon>\n\t\t\t\t\t<state>open closed error fetching0 fetching1 fetching2</state>\n");
				fprintf(kmlsnapfile, "\t\t\t\t\t<href>http://maps.google.com/mapfiles/kml/shapes/camera-lv.png</href>\n\t\t\t\t</ItemIcon>\n\t\t\t\t<bgColor>00ffffff</bgColor>\n\t\t\t\t<maxSnippetLines>2</maxSnippetLines>\n");
				fprintf(kmlsnapfile, "\t\t\t</ListStyle>\n\t\t</Style>\n");
				fprintf(kmlsnapfile, "\t\t<Icon>\n\t\t\t<href>%.255s</href>\n\t\t</Icon>\n", snapfilename);
				fprintf(kmlsnapfile, "\t\t<ViewVolume>\n\t\t\t<leftFov>-25</leftFov>\n\t\t\t<rightFov>25</rightFov>\n\t\t\t<bottomFov>-16.25</bottomFov>\n\t\t\t<topFov>16.25</topFov>\n\t\t\t<near>7.92675</near>\n\t\t</ViewVolume>\n");
				fprintf(kmlsnapfile, "\t\t<Point>\n\t\t\t<altitudeMode>relativeToGround</altitudeMode>\n\t\t\t<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode>\n\t\t\t<coordinates>%.8f,%.8f,%.3f</coordinates>\n\t\t</Point>\n", d1, d0, d2);
				fprintf(kmlsnapfile, "\t</PhotoOverlay>\n");
				fprintf(kmlsnapfile, "</Document>\n</kml>\n");
				fclose(kmlsnapfile);
			}
			printf("Snapshot.\n");
			break;
		case 'r':
			if (bVideoRecording)
			{
				EnterCriticalSection(&VideoRecordRequestsCS[videoid]);
				VideoRecordRequests[videoid] = 0; // Force recording to stop.
				LeaveCriticalSection(&VideoRecordRequestsCS[videoid]);
				bVideoRecording = FALSE;
				bDispRecordSymbol = FALSE;
			}
			else
			{
				EnterCriticalSection(&VideoRecordRequestsCS[videoid]);
				VideoRecordRequests[videoid] = 1; // Force recording to start.
				LeaveCriticalSection(&VideoRecordRequestsCS[videoid]);
				bVideoRecording = TRUE;
				bDispRecordSymbol = TRUE;
				StartChrono(&chrono_recording);
			}
			break;
		case 'p':
			if (bMissionRunning)
			{
				AbortMission();
				bDispPlaySymbol = FALSE;

				//if (bMissionPaused)
				//{
				//	bMissionPaused = FALSE;
				//	ResumeMission();
				//	bDispPauseSymbol = FALSE;
				//}
				//else
				//{
				//	bMissionPaused = TRUE;
				//	PauseMission();
				//	bDispPauseSymbol = TRUE;
				//	StartChrono(&chrono_pausing);
				//}

			}
			else
			{
				CallMission("mission.txt");
				bDispPlaySymbol = TRUE;
				StartChrono(&chrono_playing);
			}
			break;
		case 'x':
			AbortMission();
			break;
		case 'h': DisplayHelp(); break;
		case '!':
			bDisableAllAlarms = !bDisableAllAlarms;
			if (bDisableAllAlarms) printf("Alarms disabled.\n");
			else printf("Alarms enabled.\n");
			break;
		case '?': bDispEPU = !bDispEPU; break;
		case '.':
			bRearmAutopilot = TRUE;
			printf("Rearm.\n");
			break;
		case '0':
			bForceDisarmAutopilot = TRUE;
			printf("Disarm.\n");
			break;
		case 'F':
			bEnableAltRCMode = !bEnableAltRCMode;
			if (bEnableAltRCMode) printf("Alternate RC mode enabled.\n");
			else printf("Alternate RC mode disabled.\n");
			break;
		case '#':
			bEnableSimulatedDVL = !bEnableSimulatedDVL;
			if (bEnableSimulatedDVL) printf("Simulated DVL enabled.\n");
			else printf("Simulated DVL disabled.\n");
			break;
		case 'X':
			bEnableOpenCVGUIs[videoid] = FALSE;
			break;
#pragma region EXTENDED MENU
		case '1':
			if (bColorsExtendedMenu) CvCycleColors(&colortextid, &colortext, CV_RGB(0, 255, 128));
			else if (bOtherOSDOptionsExtendedMenu) bDispLLA = !bDispLLA;
			else if (bExtendedMenu) bColorsExtendedMenu = TRUE;
			break;
		case '2':
			if (bColorsExtendedMenu) CvCycleColors(&colorsonarlidarid, &colorsonarlidar, CV_RGB(0, 0, 255));
			else if (bCISCREAOSDExtendedMenu) { OSDButtonCISCREA = 'D'; bOSDButtonPressedCISCREA = TRUE; }
			else if (bOtherOSDOptionsExtendedMenu) bDispAltitudeAGL = !bDispAltitudeAGL;
			else if (bExtendedMenu) bCISCREAOSDExtendedMenu = TRUE;
			break;
		case '3':
			if (bColorsExtendedMenu) CvCycleColors(&colormapid, &colormap, CV_RGB(0, 255, 0));
			else if (bOtherOSDOptionsExtendedMenu) bDispSOG = !bDispSOG;
			else if (bExtendedMenu) bStdOutDetailedInfo = !bStdOutDetailedInfo;
			break;
		case '4': 
			if (bCISCREAOSDExtendedMenu) { OSDButtonCISCREA = 'L'; bOSDButtonPressedCISCREA = TRUE; } 
			else if (bOtherOSDOptionsExtendedMenu) bDispYPR = !bDispYPR;
			else if (bExtendedMenu)
			{
				bDisableRollWindCorrectionSailboat = !bDisableRollWindCorrectionSailboat;
				if (bDisableRollWindCorrectionSailboat) printf("Sailboat roll wind correction disabled.\n");
				else printf("Sailboat roll wind correction enabled.\n");
			}
			break;
		case '5': 
			if (bCISCREAOSDExtendedMenu) { OSDButtonCISCREA = 'S'; bOSDButtonPressedCISCREA = TRUE; } 
			else if (bExtendedMenu)
			{
				bEnableBackwardsMotorboat = !bEnableBackwardsMotorboat;
				if (bEnableBackwardsMotorboat) printf("Motorboat backwards enabled.\n");
				else printf("Motorboat backwards disabled.\n");
			}
			break;
		case '6': 
			if (bCISCREAOSDExtendedMenu) { OSDButtonCISCREA = 'R'; bOSDButtonPressedCISCREA = TRUE; } 
			else if (bExtendedMenu)
			{
				LoadKeys();
				printf("Keys updated.\n");
				DisplayKeys();
			}
			break;
		case '7': 
			if (bExtendedMenu) bOtherOSDOptionsExtendedMenu = TRUE;
			break;
		case '8': if (bCISCREAOSDExtendedMenu) { OSDButtonCISCREA = 'U'; bOSDButtonPressedCISCREA = TRUE; } break;
		case 13: // ENTER
			if (bExtendedMenu) 
			{
				if (bColorsExtendedMenu) bColorsExtendedMenu = FALSE;
				else if (bCISCREAOSDExtendedMenu) bCISCREAOSDExtendedMenu = FALSE;
				else if (bOtherOSDOptionsExtendedMenu) bOtherOSDOptionsExtendedMenu = FALSE;
				else bExtendedMenu = FALSE;
			}
			else bExtendedMenu = TRUE;
			break;
#pragma endregion
		case 27: // ESC
			bExit = TRUE;
			break;
		case 'E':
			cap = 0.0; // est
			wpsi = 0.0; // est
			break;
		case 'W':
			cap = M_PI; // ouest
			wpsi = M_PI; // ouest
			break;
		case 'K':
			cap = M_PI / 2; //nord
			wpsi = M_PI / 2; //nord
			break;
		case 'L':
			cap = -M_PI / 2; //sud
			wpsi = -M_PI / 2; //sud
			break;
		default:
			if (bZQSDPressed)
			{
				bZQSDPressed = FALSE;
				if (bEnableAltRCMode)
				{
					switch (robid)
					{
					case SAILBOAT_SIMULATOR_ROBID:
					case VAIMOS_ROBID:
					case SAILBOAT_ROBID:
					case SAILBOAT2_ROBID:
						if (!bHeadingControl) uw = 0;
						break;
					case MOTORBOAT_SIMULATOR_ROBID:
					case MOTORBOAT_ROBID:
					case BUGGY_SIMULATOR_ROBID:
					case BUGGY_ROBID:
						u = 0;
						break;
					default:
						u = 0;
						if (!bHeadingControl) uw = 0;
						break;
					}
				}
			}
			break;
		}
#pragma endregion
		EnterCriticalSection(&dispimgsCS[videoid]);
#pragma region EXTENDED MENU
		if ((bExtendedMenu)&&(!bCISCREAOSDExtendedMenu))
		{
			if (bColorsExtendedMenu)
			{
				offset = 0;
				offset += 16;
				strcpy(szText, "EXTENDED MENU\\COLORS");
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				sprintf(szText, "[1] TEXT (%03u,%03u,%03u)", (unsigned int)colortext.val[2], (unsigned int)colortext.val[1], (unsigned int)colortext.val[0]);
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				sprintf(szText, "[2] SONAR/LIDAR (%03u,%03u,%03u)", (unsigned int)colorsonarlidar.val[2], (unsigned int)colorsonarlidar.val[1], (unsigned int)colorsonarlidar.val[0]);
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				sprintf(szText, "[3] MAP (%03u,%03u,%03u)", (unsigned int)colormap.val[2], (unsigned int)colormap.val[1], (unsigned int)colormap.val[0]);
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				strcpy(szText, "[ENTER] EXIT");
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
			}
			else if (bOtherOSDOptionsExtendedMenu)
			{
				offset = 0;
				offset += 16;
				strcpy(szText, "EXTENDED MENU\\OTHER OSD OPTIONS");
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				sprintf(szText, "[1] DISPLAY LLA (%d)", (int)bDispLLA);
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				sprintf(szText, "[2] DISPLAY ALTITUDE AGL (%d)", (int)bDispAltitudeAGL);
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				sprintf(szText, "[3] DISPLAY SOG (%d)", (int)bDispSOG);
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				sprintf(szText, "[4] DISPLAY YPR (%d)", (int)bDispYPR);
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				strcpy(szText, "[ENTER] EXIT");
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
			}
			else
			{
				offset = 0;
				offset += 16;
				strcpy(szText, "EXTENDED MENU");
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				strcpy(szText, "[1] COLORS");
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				strcpy(szText, "[2] CISCREA OSD (46825)");
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				sprintf(szText, "[3] SHOW DETAILED INFO (%d)", (int)bStdOutDetailedInfo);
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				sprintf(szText, "[4] SAILBOAT ROLL WIND CORR (%d)", (int)!bDisableRollWindCorrectionSailboat);
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				sprintf(szText, "[5] MOTORBOAT BACKWARDS (%d)", (int)bEnableBackwardsMotorboat);
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				strcpy(szText, "[6] LOAD KEYS");
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				strcpy(szText, "[7] OTHER OSD OPTIONS");
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				offset += 16;
				strcpy(szText, "[ENTER] EXIT");
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
			}
		}
#pragma endregion
#pragma region OSD
		if ((bOSD)&&(!bExtendedMenu))
		{
			offset = 0;
			// Rounding...
			switch (robid)
			{
			case BUBBLE_ROBID:
			case ETAS_WHEEL_ROBID:
				sprintf(szText, "%d%% %d%% %d%% %d%%", (int)floor(u_max*100.0+0.05), (int)floor(uw_max*100.0+0.05), (int)floor(u2*100.0+0.05), (int)floor(u1*100.0+0.05));
				break;
			case MOTORBOAT_SIMULATOR_ROBID:
			case MOTORBOAT_ROBID:
			case BUGGY_SIMULATOR_ROBID:
			case BUGGY_ROBID:
				sprintf(szText, "%+04d%% %+04d%% %+04d%%", (int)floor(u_max*100.0+0.05), (int)floor(uw*100.0+0.05), (int)floor(u*100.0+0.05));
				break;
			case SAILBOAT_SIMULATOR_ROBID:
			case VAIMOS_ROBID:
			case SAILBOAT_ROBID:
			case SAILBOAT2_ROBID:
				switch (state)
				{
				case DIRECT_TRAJECTORY: s = 'D'; break;
				case STARBOARD_TACK_TRAJECTORY: s = 'S'; break;
				case PORT_TACK_TRAJECTORY: s = 'P'; break;
				default: s = 'I'; break;
				}
				sprintf(szText, "%c %c %d%% %d%% VBAT1:%.1fV",
					(vswitch*vswitchcoef > vswitchthreshold? 'A': 'M'), s, (int)floor(uw*100.0+0.05), (int)floor(u*100.0+0.05), vbat1);
				break;
			case LIRMIA3_ROBID:
				sprintf(szText, "%+04d%% %+04d%% %d%% %d%% %d%% %d%%", (int)floor(uw*100.0+0.05), (int)floor(u*100.0+0.05), (int)floor(u4*100.0+0.05), (int)floor(u3*100.0+0.05), (int)floor(u2*100.0+0.05), (int)floor(u1*100.0+0.05));
				break;
			default:
				sprintf(szText, "%+04d%% %+04d%% %d%% %d%% %d%%", (int)floor(uw*100.0+0.05), (int)floor(u*100.0+0.05), (int)floor(u3*100.0+0.05), (int)floor(u2*100.0+0.05), (int)floor(u1*100.0+0.05));
				break;
			}
			offset += 16;
			cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
			if (bDispLLA)
			{
				// In deg in NED coordinate system.
				if (bHeadingControl) sprintf(szText, "%.2f/%.2f", fmod_360_pos_rad2deg(-angle_env-Center(psihat)+M_PI/2.0), fmod_360_pos_rad2deg(-angle_env-wpsi+M_PI/2.0));
				else sprintf(szText, "%.2f/--", fmod_360_pos_rad2deg(-angle_env-Center(psihat)+M_PI/2.0));
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				if (bDispYPR)
				{
					if (bPitchControl) sprintf(szText, "%.2f/%.2f", fmod_360_rad2deg(-Center(thetahat)), fmod_360_rad2deg(-wtheta));
					else sprintf(szText, "%.2f/--", fmod_360_rad2deg(-Center(thetahat)));
					offset += 16;
					cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
					if (bRollControl) sprintf(szText, "%.2f/%.2f", fmod_360_rad2deg(Center(phihat)), fmod_360_rad2deg(wphi));
					else sprintf(szText, "%.2f/--", fmod_360_rad2deg(Center(phihat)));
					offset += 16;
					cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				}
			}
			else
			{
				// In deg in the reference coordinate system.
				if (bHeadingControl) sprintf(szText, "%.2f/%.2f", fmod_360_pos_rad2deg(Center(psihat)), fmod_360_pos_rad2deg(wpsi));
				else sprintf(szText, "%.2f/--", fmod_360_pos_rad2deg(Center(psihat)));
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				if (bDispYPR)
				{
					if (bPitchControl) sprintf(szText, "%.2f/%.2f", fmod_360_rad2deg(Center(thetahat)), fmod_360_rad2deg(wtheta));
					else sprintf(szText, "%.2f/--", fmod_360_rad2deg(Center(thetahat)));
					offset += 16;
					cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
					if (bRollControl) sprintf(szText, "%.2f/%.2f", fmod_360_rad2deg(Center(phihat)), fmod_360_rad2deg(wphi));
					else sprintf(szText, "%.2f/--", fmod_360_rad2deg(Center(phihat)));
					offset += 16;
					cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				}
			}
			if (robid & SUBMARINE_ROBID_MASK) 
			{
				if (bDepthControl) sprintf(szText, "%.2f/%.2f", Center(zhat), wz);
				else sprintf(szText, "%.2f/--", Center(zhat));
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				if (bDispAltitudeAGL)
				{
					if (bAltitudeAGLControl) sprintf(szText, "AGL=%.2f/%.2f", altitude_AGL, wagl);
					else sprintf(szText, "AGL=%.2f/--", altitude_AGL);
					offset += 16;
					cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				}
			}
			if (robid & AERIAL_ROBID_MASK) 
			{
				if (bDepthControl) sprintf(szText, "%.2f/%.2f", Center(zhat), wz);
				else sprintf(szText, "%.2f/--", Center(zhat));
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				if (bDispAltitudeAGL)
				{
					if (bAltitudeAGLControl) sprintf(szText, "A_F=%.2f/%.2f", altitude_AGL, wagl);
					else sprintf(szText, "A_F=%.2f/--", altitude_AGL);
					offset += 16;
					cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				}
			}
			if (robid & SAILBOAT_CLASS_ROBID_MASK) 
			{
				sprintf(szText, "%.1f/%.1f", 
					// Apparent wind for Sailboat, true wind for VAIMOS for unfiltered value.
					(robid == SAILBOAT_ROBID)? (fmod_2PI(-psiawind+M_PI+M_PI)+M_PI)*180.0/M_PI: (fmod_2PI(-angle_env-psitwind+M_PI+3.0*M_PI/2.0)+M_PI)*180.0/M_PI, 
					(fmod_2PI(-angle_env-Center(psitwindhat)+M_PI+3.0*M_PI/2.0)+M_PI)*180.0/M_PI);
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
			}
			if (bDispLLA)
			{
				EnvCoordSystem2GPS(lat_env, long_env, alt_env, angle_env, Center(xhat), Center(yhat), Center(zhat), &d0, &d1, &d2);
				sprintf(szText, "POS:%.6f,%.6f", d0, d1);
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
			}
			else
			{
				sprintf(szText, "POS:%.2f,%.2f", Center(xhat), Center(yhat));
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
			}
			sprintf(szText, "ERR:%.2f,%.2f", Width(xhat)/2.0, Width(yhat)/2.0);
			offset += 16;
			cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
			if (bDynamicSonarLocalization)
			{
				sprintf(szText, "SNR DYN LOC");
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
			}
			if (bDispDVL)
			{
				if (bDVLLocalization)
				{
					offset += 16;
					if ((Width(vrx_dvl) <= 4*dvl_acc)&&(Width(vry_dvl) <= 4*dvl_acc))
					{
						sprintf(szText, "DVL LOC");
						cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
					}
					else
					{
						sprintf(szText, "DVL LOC N/A");
						cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, CV_RGB(255, 0, 0));
					}
				}
			}
			if (bDispGPS)
			{
				offset += 16;
				switch (GetGNSSlevel())
				{
				case GNSS_ACC_LEVEL_RTK_FIXED:
					if (bGPSLocalization) strcpy(szText, "RTK FIX (IN USE)"); else strcpy(szText, "RTK FIX");
					cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
					break;
				case GNSS_ACC_LEVEL_RTK_FLOAT:
					if (bGPSLocalization) strcpy(szText, "RTK FLT (IN USE)"); else strcpy(szText, "RTK FLT");
					cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
					break;
				case GNSS_ACC_LEVEL_RTK_UNREL:
					if (bGPSLocalization) strcpy(szText, "RTK ? (IN USE)"); else strcpy(szText, "RTK ?");
					cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, CV_RGB(255, 128, 0));
					break;
				case GNSS_ACC_LEVEL_GNSS_FIX_HIGH:
					if (bGPSLocalization) strcpy(szText, "GPS HIGH (IN USE)"); else strcpy(szText, "GPS HIGH");
					cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
					break;
				case GNSS_ACC_LEVEL_GNSS_FIX_MED:
					if (bGPSLocalization) strcpy(szText, "GPS MED (IN USE)"); else strcpy(szText, "GPS MED");
					cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
					break;
				case GNSS_ACC_LEVEL_GNSS_FIX_LOW:
					if (bGPSLocalization) strcpy(szText, "GPS LOW (IN USE)"); else strcpy(szText, "GPS LOW");
					cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
					break;
				case GNSS_ACC_LEVEL_GNSS_FIX_UNREL:
					if (bGPSLocalization) strcpy(szText, "GPS ? (IN USE)"); else strcpy(szText, "GPS ?");
					cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, CV_RGB(255, 128, 0));
					break;
				default:
					strcpy(szText, "NO FIX");
					cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, CV_RGB(255, 0, 0));
					break;
				}
			}
			if (bDispSOG)
			{
				sprintf(szText, "SOG:%.2f", sog);
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
				sprintf(szText, "COG:%.2f", (fmod_2PI(-angle_env-cog+3.0*M_PI/2.0)+M_PI)*180.0/M_PI);
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0, offset), &font, colortext);
			}
			if (bDispEPU)
			{
				if (GetTimeElapsedChronoQuick(&chrono_epu) < 2.0)
				{
					sprintf(szText, "VBAT1:%.1fV, VBAT2:%.1fV", vbat1, vbat2);
				}
				else if (GetTimeElapsedChronoQuick(&chrono_epu) < 4.0)
				{
					sprintf(szText, "VFBAT1:%.1fV, VFBAT2:%.1fV", vbat1_filtered, vbat2_filtered);
				}
				else if (GetTimeElapsedChronoQuick(&chrono_epu) < 6.0)
				{
					sprintf(szText, "IBAT1:%.1fA, IBAT2:%.1fA", ibat1, ibat2);
				}
				else if (GetTimeElapsedChronoQuick(&chrono_epu) < 8.0)
				{
					sprintf(szText, "IFBAT1:%.1fA, IFBAT2:%.1fA", ibat1_filtered, ibat2_filtered);
				}
				else if (GetTimeElapsedChronoQuick(&chrono_epu) < 10.0)
				{
					sprintf(szText, "EPU1:%.1fWh, EPU2:%.1fWh", Energy_electronics, Energy_actuators);
				}
				else
				{
					StopChronoQuick(&chrono_epu);
					sprintf(szText, "VBAT1:%.1fV, VBAT2:%.1fV", vbat1, vbat2);
					StartChrono(&chrono_epu);
				}
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
			}
			if (bWaypointControl)
			{
				if (bDispLLA)
				{
					EnvCoordSystem2GPS(lat_env, long_env, alt_env, angle_env, wx, wy, wz, &d0, &d1, &d2);
					sprintf(szText, "WPT:%.6f,%.6f", d0, d1);
					offset += 16;
					cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				}
				else
				{
					sprintf(szText, "WPT:%.2f,%.2f", wx, wy);
					offset += 16;
					cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				}
				d0 = sqrt(pow(wx-Center(xhat),2)+pow(wy-Center(yhat),2));
				sprintf(szText, "DIS:%.2f", d0);
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				if (bDispSOG)
				{
					if (sog > 0) d1 = d0/sog; else d1 = 0;
					DecSec2DaysHoursMinSec(d1, &days, &hours, &minutes, &seconds, &deccsec);
					sprintf(szText, "ETR:%02d:%02d:%02d", (int)(days*24+hours), minutes, seconds);
					offset += 16;
					cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				}
			}
			if (bLineFollowingControl)
			{
				if (bDispLLA)
				{
					EnvCoordSystem2GPS(lat_env, long_env, alt_env, angle_env, wxb, wyb, wz, &d0, &d1, &d2);
					sprintf(szText, "WPT:%.6f,%.6f", d0, d1);
					offset += 16;
					cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				}
				else
				{
					sprintf(szText, "WPT:%.2f,%.2f", wxb, wyb);
					offset += 16;
					cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				}
				sprintf(szText, "XTE:%.2f", xte);
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				d0 = sqrt(pow(wxb-Center(xhat),2)+pow(wyb-Center(yhat),2));
				sprintf(szText, "DIS:%.2f", d0);
				offset += 16;
				cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				if (bDispSOG)
				{
					if (sog > 0) d1 = d0/sog; else d1 = 0;
					DecSec2DaysHoursMinSec(d1, &days, &hours, &minutes, &seconds, &deccsec);
					sprintf(szText, "ETR:%02d:%02d:%02d", (int)(days*24+hours), minutes, seconds);
					offset += 16;
					cvPutText(dispimgs[videoid], szText, cvPoint(0,offset), &font, colortext);
				}
			}
			if (bOrientationCircle)
			{
				cvCircle(dispimgs[videoid], cvPoint(videoimgwidth-16, 32), 12, CV_RGB(255, 0, 0), 2, 8, 0);
				if (robid & SAILBOAT_CLASS_ROBID_MASK) 
				{
					angle = M_PI/2.0+Center(psitwindhat)+M_PI-Center(psihat);
					cvLine(dispimgs[videoid], cvPoint(videoimgwidth-16, 32), 
						cvPoint((int)(videoimgwidth-16+12*cos(angle)), (int)(32-12*sin(angle))), 
						CV_RGB(0, 255, 255), 2, 8, 0);
				}
				if (bHeadingControl) 
				{
					angle = M_PI/2.0+wpsi-Center(psihat);
					cvLine(dispimgs[videoid], cvPoint(videoimgwidth-16, 32), 
						cvPoint((int)(videoimgwidth-16+12*cos(angle)), (int)(32-12*sin(angle))), 
						CV_RGB(0, 255, 0), 2, 8, 0);
				}
				angle = M_PI-angle_env-Center(psihat);
				cvLine(dispimgs[videoid], cvPoint(videoimgwidth-16, 32), 
					cvPoint((int)(videoimgwidth-16+12*cos(angle)), (int)(32-12*sin(angle))), 
					CV_RGB(0, 0, 255), 2, 8, 0);
			}
			if (bMap)
			{
				int detailswidth = 96, detailsheight = 96;
				int detailsj = videoimgwidth-detailswidth-8, detailsi = 48;
				InitCS2ImgEx(&csMap2FullImg, &csMap, detailswidth, detailsheight, BEST_RATIO_COORDSYSTEM2IMG);
				cvRectangle(dispimgs[videoid], 
					cvPoint(detailsj+detailswidth, detailsi+detailsheight), cvPoint(detailsj-1, detailsi-1), 
					CV_RGB(255, 255, 255), 1, 8, 0);
				cvRectangle(dispimgs[videoid], 
					cvPoint(detailsj+detailswidth-1, detailsi+detailsheight-1), cvPoint(detailsj, detailsi), 
					CV_RGB(0, 0, 0), 1, 8, 0);
				if (bRotatingMap)
				{
					// Environment circles.
					for (i = 0; i < nb_circles; i++)
					{
						cvCircle(dispimgs[videoid], 
							cvPoint(
							detailsj+XCS2JImg(&csMap2FullImg, (circles_x[i]-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(circles_y[i]-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							detailsi+YCS2IImg(&csMap2FullImg, (circles_x[i]-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(circles_y[i]-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							(int)(circles_r[i]*csMap2FullImg.JXRatio), colormap, 1, 8, 0);
					}
					// Environment walls.
					for (i = 0; i < nb_walls; i++)
					{
						cvLine(dispimgs[videoid], 
							cvPoint(
							detailsj+XCS2JImg(&csMap2FullImg, (walls_xa[i]-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(walls_ya[i]-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							detailsi+YCS2IImg(&csMap2FullImg, (walls_xa[i]-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(walls_ya[i]-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							cvPoint(
							detailsj+XCS2JImg(&csMap2FullImg, (walls_xb[i]-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(walls_yb[i]-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							detailsi+YCS2IImg(&csMap2FullImg, (walls_xb[i]-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(walls_yb[i]-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							colormap, 1, 8, 0);
					}
					// Waypoint.
					if (bWaypointControl)
					{
						cvCircle(dispimgs[videoid], 
							cvPoint(
							detailsj+XCS2JImg(&csMap2FullImg, (wx-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wy-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							detailsi+YCS2IImg(&csMap2FullImg, (wx-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wy-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(255, 255, 255), 8, 8, 0);
						cvCircle(dispimgs[videoid], 
							cvPoint(
							detailsj+XCS2JImg(&csMap2FullImg, (wx-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wy-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							detailsi+YCS2IImg(&csMap2FullImg, (wx-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wy-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(0, 0, 255), 4, 8, 0);
					}
					if (bLineFollowingControl)
					{
						cvLine(dispimgs[videoid], 
							cvPoint(
							detailsj+XCS2JImg(&csMap2FullImg, (wxa-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wya-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							detailsi+YCS2IImg(&csMap2FullImg, (wxa-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wya-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							cvPoint(
							detailsj+XCS2JImg(&csMap2FullImg, (wxb-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wyb-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							detailsi+YCS2IImg(&csMap2FullImg, (wxb-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wyb-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							CV_RGB(255, 255, 255), 2, 8, 0);
						cvLine(dispimgs[videoid], 
							cvPoint(
							detailsj+XCS2JImg(&csMap2FullImg, (wxa-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wya-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							detailsi+YCS2IImg(&csMap2FullImg, (wxa-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wya-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							cvPoint(
							detailsj+XCS2JImg(&csMap2FullImg, (wxb-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wyb-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							detailsi+YCS2IImg(&csMap2FullImg, (wxb-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wyb-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							CV_RGB(0, 0, 255), 1, 8, 0);
						cvCircle(dispimgs[videoid], 
							cvPoint(
							detailsj+XCS2JImg(&csMap2FullImg, (wxb-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wyb-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							detailsi+YCS2IImg(&csMap2FullImg, (wxb-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wyb-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(255, 255, 255), 8, 8, 0);
						cvCircle(dispimgs[videoid], 
							cvPoint(
							detailsj+XCS2JImg(&csMap2FullImg, (wxb-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wyb-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							detailsi+YCS2IImg(&csMap2FullImg, (wxb-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wyb-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(0, 0, 255), 4, 8, 0);
					}
					// Robot.
					switch (robid)
					{
					case BUBBLE_ROBID:
					case ETAS_WHEEL_ROBID:
					case MOTORBOAT_SIMULATOR_ROBID:
					case MOTORBOAT_ROBID:
					case BUGGY_SIMULATOR_ROBID:
					case BUGGY_ROBID:
					default:
						cvLine(dispimgs[videoid], 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, 0), detailsi+YCS2IImg(&csMap2FullImg, -0.4)), 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, 0), detailsi+YCS2IImg(&csMap2FullImg, 0.0)), 
							CV_RGB(255, 128, 128), 4, 8, 0);
						cvLine(dispimgs[videoid], 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, 0), detailsi+YCS2IImg(&csMap2FullImg, 0.0)), 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, 0), detailsi+YCS2IImg(&csMap2FullImg, 0.4)), 
							CV_RGB(0, 255, 0), 4, 8, 0);
						break;
					}
				}
				else
				{
					// Environment circles.
					for (i = 0; i < nb_circles; i++)
					{
						cvCircle(dispimgs[videoid], 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, circles_x[i]), detailsi+YCS2IImg(&csMap2FullImg, circles_y[i])), 
							(int)(circles_r[i]*csMap2FullImg.JXRatio), colormap, 1, 8, 0);
					}
					// Environment walls.
					for (i = 0; i < nb_walls; i++)
					{
						cvLine(dispimgs[videoid], 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, walls_xa[i]), detailsi+YCS2IImg(&csMap2FullImg, walls_ya[i])), 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, walls_xb[i]), detailsi+YCS2IImg(&csMap2FullImg, walls_yb[i])), 
							colormap, 1, 8, 0);
					}
					// Waypoint.
					if (bWaypointControl)
					{
						cvCircle(dispimgs[videoid], 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, wx), detailsi+YCS2IImg(&csMap2FullImg, wy)), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(255, 255, 255), 8, 8, 0);
						cvCircle(dispimgs[videoid], 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, wx), detailsi+YCS2IImg(&csMap2FullImg, wy)), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(0, 0, 255), 4, 8, 0);
					}
					if (bLineFollowingControl)
					{
						cvLine(dispimgs[videoid], 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, wxa), detailsi+YCS2IImg(&csMap2FullImg, wya)), 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, wxb), detailsi+YCS2IImg(&csMap2FullImg, wyb)), 
							CV_RGB(255, 255, 255), 2, 8, 0);
						cvLine(dispimgs[videoid], 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, wxa), detailsi+YCS2IImg(&csMap2FullImg, wya)), 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, wxb), detailsi+YCS2IImg(&csMap2FullImg, wyb)), 
							CV_RGB(0, 0, 255), 1, 8, 0);
						cvCircle(dispimgs[videoid], 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, wxb), detailsi+YCS2IImg(&csMap2FullImg, wyb)), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(255, 255, 255), 8, 8, 0);
						cvCircle(dispimgs[videoid], 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, wxb), detailsi+YCS2IImg(&csMap2FullImg, wyb)), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(0, 0, 255), 4, 8, 0);
					}
					// Robot.
					switch (robid)
					{
					case BUBBLE_ROBID:
					case ETAS_WHEEL_ROBID:
					case MOTORBOAT_SIMULATOR_ROBID:
					case MOTORBOAT_ROBID:
					case BUGGY_SIMULATOR_ROBID:
					case BUGGY_ROBID:
					default:
						cvLine(dispimgs[videoid], 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, Center(xhat)-0.4*cos(Center(psihat))), detailsi+YCS2IImg(&csMap2FullImg, Center(yhat)-0.4*sin(Center(psihat)))), 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, Center(xhat)+0.0*cos(Center(psihat))), detailsi+YCS2IImg(&csMap2FullImg, Center(yhat)+0.0*sin(Center(psihat)))), 
							CV_RGB(255, 128, 0), 4, 8, 0);
						cvLine(dispimgs[videoid], 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, Center(xhat)-0.0*cos(Center(psihat))), detailsi+YCS2IImg(&csMap2FullImg, Center(yhat)-0.0*sin(Center(psihat)))), 
							cvPoint(detailsj+XCS2JImg(&csMap2FullImg, Center(xhat)+0.4*cos(Center(psihat))), detailsi+YCS2IImg(&csMap2FullImg, Center(yhat)+0.4*sin(Center(psihat)))), 
							CV_RGB(0, 255, 0), 4, 8, 0);
						break;
					}
				}
			}
			if (bFullMap)
			{
				InitCS2ImgEx(&csMap2FullImg, &csMap, videoimgwidth, videoimgheight, BEST_RATIO_COORDSYSTEM2IMG);
				if (bRotatingMap)
				{
					// Environment circles.
					for (i = 0; i < nb_circles; i++)
					{
						cvCircle(dispimgs[videoid], 
							cvPoint(
							XCS2JImg(&csMap2FullImg, (circles_x[i]-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(circles_y[i]-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							YCS2IImg(&csMap2FullImg, (circles_x[i]-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(circles_y[i]-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							(int)(circles_r[i]*csMap2FullImg.JXRatio), colormap, 2, 8, 0);
					}
					// Environment walls.
					for (i = 0; i < nb_walls; i++)
					{
						cvLine(dispimgs[videoid], 
							cvPoint(
							XCS2JImg(&csMap2FullImg, (walls_xa[i]-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(walls_ya[i]-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							YCS2IImg(&csMap2FullImg, (walls_xa[i]-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(walls_ya[i]-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							cvPoint(
							XCS2JImg(&csMap2FullImg, (walls_xb[i]-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(walls_yb[i]-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							YCS2IImg(&csMap2FullImg, (walls_xb[i]-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(walls_yb[i]-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							colormap, 2, 8, 0);
					}
					// Waypoint.
					if (bWaypointControl)
					{
						cvCircle(dispimgs[videoid], 
							cvPoint(
							XCS2JImg(&csMap2FullImg, (wx-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wy-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							YCS2IImg(&csMap2FullImg, (wx-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wy-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(255, 255, 255), 8, 8, 0);
						cvCircle(dispimgs[videoid], 
							cvPoint(
							XCS2JImg(&csMap2FullImg, (wx-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wy-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							YCS2IImg(&csMap2FullImg, (wx-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wy-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(0, 0, 255), 4, 8, 0);
					}
					if (bLineFollowingControl)
					{
						cvLine(dispimgs[videoid], 
							cvPoint(
							XCS2JImg(&csMap2FullImg, (wxa-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wya-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							YCS2IImg(&csMap2FullImg, (wxa-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wya-Center(yhat))*cos(M_PI/2.0-Center(psihat)))), 
							cvPoint(
							XCS2JImg(&csMap2FullImg, (wxb-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wyb-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							YCS2IImg(&csMap2FullImg, (wxb-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wyb-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							CV_RGB(255, 255, 255), 2, 8, 0);
						cvLine(dispimgs[videoid], 
							cvPoint(
							XCS2JImg(&csMap2FullImg, (wxa-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wya-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							YCS2IImg(&csMap2FullImg, (wxa-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wya-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							cvPoint(
							XCS2JImg(&csMap2FullImg, (wxb-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wyb-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							YCS2IImg(&csMap2FullImg, (wxb-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wyb-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							CV_RGB(0, 0, 255), 1, 8, 0);
						cvCircle(dispimgs[videoid], 
							cvPoint(
							XCS2JImg(&csMap2FullImg, (wxb-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wyb-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							YCS2IImg(&csMap2FullImg, (wxb-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wyb-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(255, 255, 255), 8, 8, 0);
						cvCircle(dispimgs[videoid], 
							cvPoint(
							XCS2JImg(&csMap2FullImg, (wxb-Center(xhat))*cos(M_PI/2.0-Center(psihat))-(wyb-Center(yhat))*sin(M_PI/2.0-Center(psihat))), 
							YCS2IImg(&csMap2FullImg, (wxb-Center(xhat))*sin(M_PI/2.0-Center(psihat))+(wyb-Center(yhat))*cos(M_PI/2.0-Center(psihat)))
							), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(0, 0, 255), 4, 8, 0);
					}
					// Robot.
					switch (robid)
					{
					case BUBBLE_ROBID:
					case ETAS_WHEEL_ROBID:
					case MOTORBOAT_SIMULATOR_ROBID:
					case MOTORBOAT_ROBID:
					case BUGGY_SIMULATOR_ROBID:
					case BUGGY_ROBID:
					default:
						cvLine(dispimgs[videoid], 
							cvPoint(XCS2JImg(&csMap2FullImg, 0), YCS2IImg(&csMap2FullImg, -0.4)), 
							cvPoint(XCS2JImg(&csMap2FullImg, 0), YCS2IImg(&csMap2FullImg, 0.0)), 
							CV_RGB(255, 128, 0), 8, 8, 0);
						cvLine(dispimgs[videoid], 
							cvPoint(XCS2JImg(&csMap2FullImg, 0), YCS2IImg(&csMap2FullImg, -0.0)), 
							cvPoint(XCS2JImg(&csMap2FullImg, 0), YCS2IImg(&csMap2FullImg, 0.4)), 
							CV_RGB(0, 255, 0), 8, 8, 0);
						break;
					}
				}
				else
				{
					// Environment circles.
					for (i = 0; i < nb_circles; i++)
					{
						cvCircle(dispimgs[videoid], 
							cvPoint(XCS2JImg(&csMap2FullImg, circles_x[i]), YCS2IImg(&csMap2FullImg, circles_y[i])), 
							(int)(circles_r[i]*csMap2FullImg.JXRatio), colormap, 2, 8, 0);
					}
					// Environment walls.
					for (i = 0; i < nb_walls; i++)
					{
						cvLine(dispimgs[videoid], 
							cvPoint(XCS2JImg(&csMap2FullImg, walls_xa[i]), YCS2IImg(&csMap2FullImg, walls_ya[i])), 
							cvPoint(XCS2JImg(&csMap2FullImg, walls_xb[i]), YCS2IImg(&csMap2FullImg, walls_yb[i])), 
							colormap, 2, 8, 0);
					}
					// Waypoint.
					if (bWaypointControl)
					{
						cvCircle(dispimgs[videoid], 
							cvPoint(XCS2JImg(&csMap2FullImg, wx), YCS2IImg(&csMap2FullImg, wy)), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(255, 255, 255), 8, 8, 0);
						cvCircle(dispimgs[videoid], 
							cvPoint(XCS2JImg(&csMap2FullImg, wx), YCS2IImg(&csMap2FullImg, wy)), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(0, 0, 255), 4, 8, 0);
					}
					if (bLineFollowingControl)
					{
						cvLine(dispimgs[videoid], 
							cvPoint(XCS2JImg(&csMap2FullImg, wxa), YCS2IImg(&csMap2FullImg, wya)), 
							cvPoint(XCS2JImg(&csMap2FullImg, wxb), YCS2IImg(&csMap2FullImg, wyb)), 
							CV_RGB(255, 255, 255), 2, 8, 0);
						cvLine(dispimgs[videoid], 
							cvPoint(XCS2JImg(&csMap2FullImg, wxa), YCS2IImg(&csMap2FullImg, wya)), 
							cvPoint(XCS2JImg(&csMap2FullImg, wxb), YCS2IImg(&csMap2FullImg, wyb)), 
							CV_RGB(0, 0, 255), 1, 8, 0);
						cvCircle(dispimgs[videoid], 
							cvPoint(XCS2JImg(&csMap2FullImg, wxb), YCS2IImg(&csMap2FullImg, wyb)), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(255, 255, 255), 8, 8, 0);
						cvCircle(dispimgs[videoid], 
							cvPoint(XCS2JImg(&csMap2FullImg, wxb), YCS2IImg(&csMap2FullImg, wyb)), 
							(int)(0.8*csMap2FullImg.JXRatio), CV_RGB(0, 0, 255), 4, 8, 0);
					}
					// Robot.
					switch (robid)
					{
					case BUGGY_SIMULATOR_ROBID:
					case BUGGY_ROBID:
						{
							//CvPoint pts[10]; int npts = sizeof(pts); int contours = 0;
							//pts[0] = cvPoint(videoimgwidth-28, 8-5);
						
							////cvPolyLine(dispimgs[videoid], &pts, &npts, contours, 1, CV_RGB(0, 255, 0), 1, 8, 0);
							//cvFillConvexPoly(dispimgs[videoid], pts, npts, CV_RGB(0, 255, 0), 8, 0);
							//break;
						}
					case BUBBLE_ROBID:
					case ETAS_WHEEL_ROBID:
					case MOTORBOAT_SIMULATOR_ROBID:
					case MOTORBOAT_ROBID:
					default:
						cvLine(dispimgs[videoid], 
							cvPoint(XCS2JImg(&csMap2FullImg, Center(xhat)-0.4*cos(Center(psihat))), YCS2IImg(&csMap2FullImg, Center(yhat)-0.4*sin(Center(psihat)))), 
							cvPoint(XCS2JImg(&csMap2FullImg, Center(xhat)+0.0*cos(Center(psihat))), YCS2IImg(&csMap2FullImg, Center(yhat)+0.0*sin(Center(psihat)))), 
							CV_RGB(255, 128, 0), 8, 8, 0);
						cvLine(dispimgs[videoid], 
							cvPoint(XCS2JImg(&csMap2FullImg, Center(xhat)-0.0*cos(Center(psihat))), YCS2IImg(&csMap2FullImg, Center(yhat)-0.0*sin(Center(psihat)))), 
							cvPoint(XCS2JImg(&csMap2FullImg, Center(xhat)+0.4*cos(Center(psihat))), YCS2IImg(&csMap2FullImg, Center(yhat)+0.4*sin(Center(psihat)))), 
							CV_RGB(0, 255, 0), 8, 8, 0);
						break;
					}
				}
			}
			EnterCriticalSection(&VideoRecordRequestsCS[videoid]);
			if (VideoRecordRequests[videoid] > 0)
			{
				LeaveCriticalSection(&VideoRecordRequestsCS[videoid]);
				if (GetTimeElapsedChronoQuick(&chrono_recording) > 0.5)
				{
					StopChronoQuick(&chrono_recording);
					bDispRecordSymbol = !bDispRecordSymbol;
					StartChrono(&chrono_recording);
				}
				if (bDispRecordSymbol) cvCircle(dispimgs[videoid], cvPoint(videoimgwidth-8, 8), 6, CV_RGB(255, 0, 0), CV_FILLED, 8, 0);
			}
			else
			{
				LeaveCriticalSection(&VideoRecordRequestsCS[videoid]);
			}
			if (bMissionRunning)
			{
				if (GetTimeElapsedChronoQuick(&chrono_playing) > 0.5)
				{
					StopChronoQuick(&chrono_playing);
					bDispPlaySymbol = !bDispPlaySymbol;
					StartChrono(&chrono_playing);
				}
				if (bDispPlaySymbol) 
				{
					nbPlaySymbolPoints = 3;
					PlaySymbolPoints[0] = cvPoint(videoimgwidth-28, 8-5);
					PlaySymbolPoints[1] = cvPoint(videoimgwidth-28, 8+5);
					PlaySymbolPoints[2] = cvPoint(videoimgwidth-18, 8);
					cvFillConvexPoly(dispimgs[videoid], PlaySymbolPoints, nbPlaySymbolPoints, CV_RGB(0, 255, 0), 8, 0);
				}
				//if (GetTimeElapsedChronoQuick(&chrono_pausing) > 0.5)
				//{
				//	StopChronoQuick(&chrono_pausing);
				//	bDispPauseSymbol = !bDispPauseSymbol;
				//	StartChrono(&chrono_pausing);
				//}
				if (bDispPauseSymbol) 
				{
					nbPauseSymbolPoints = 4;
					PauseSymbolPoints[0] = cvPoint(videoimgwidth-28, 8-5);
					PauseSymbolPoints[1] = cvPoint(videoimgwidth-28, 8+5);
					PauseSymbolPoints[2] = cvPoint(videoimgwidth-25, 8+5);
					PauseSymbolPoints[3] = cvPoint(videoimgwidth-25, 8-5);
					cvFillConvexPoly(dispimgs[videoid], PauseSymbolPoints, nbPauseSymbolPoints, CV_RGB(0, 255, 0), 8, 0);
					nbPauseSymbolPoints = 4;
					PauseSymbolPoints[0] = cvPoint(videoimgwidth-21, 8-5);
					PauseSymbolPoints[1] = cvPoint(videoimgwidth-21, 8+5);
					PauseSymbolPoints[2] = cvPoint(videoimgwidth-18, 8+5);
					PauseSymbolPoints[3] = cvPoint(videoimgwidth-18, 8-5);
					cvFillConvexPoly(dispimgs[videoid], PauseSymbolPoints, nbPauseSymbolPoints, CV_RGB(0, 255, 0), 8, 0);
				}
			}
			if (!bDisableAllAlarms)
			{
				if (GetTimeElapsedChronoQuick(&chrono_alarms) < 1.0)
				{
				}
				else if (GetTimeElapsedChronoQuick(&chrono_alarms) < 2.0)
				{
					if ((vbat1_threshold > 0.01)&&(vbat1_filtered < vbat1_threshold))
					{
						strcpy(szText, "BAT1 ALARM");
						cvPutText(dispimgs[videoid], szText, cvPoint(videoimgwidth-16*8, videoimgheight-8-3*16), &font, CV_RGB(255, 0, 0));
					}
				}
				else if (GetTimeElapsedChronoQuick(&chrono_alarms) < 3.0)
				{
					if ((vbat2_threshold > 0.01)&&(vbat2_filtered < vbat2_threshold))
					{
						strcpy(szText, "BAT2 ALARM");
						cvPutText(dispimgs[videoid], szText, cvPoint(videoimgwidth-16*8, videoimgheight-8-3*16), &font, CV_RGB(255, 0, 0));
					}
				}
				else if (GetTimeElapsedChronoQuick(&chrono_alarms) < 4.0)
				{
				}
				else if (GetTimeElapsedChronoQuick(&chrono_alarms) < 5.0)
				{
				}
				else
				{
					StopChronoQuick(&chrono_alarms);
					StartChrono(&chrono_alarms);
				}
			}
		}
#pragma endregion
		LeaveCriticalSection(&dispimgsCS[videoid]);
		LeaveCriticalSection(&StateVariablesCS);

		EnterCriticalSection(&dispimgsCS[videoid]);
#ifndef USE_OPENCV_HIGHGUI_CPP_API
		cvShowImage(windowname, dispimgs[videoid]);
#else
		cv::imshow(windowname, cv::cvarrToMat(dispimgs[videoid]));
#endif // !USE_OPENCV_HIGHGUI_CPP_API
		LeaveCriticalSection(&dispimgsCS[videoid]);

		if (bExit) break;
	}

	if (bVideoRecording)
	{
		EnterCriticalSection(&VideoRecordRequestsCS[videoid]);
		VideoRecordRequests[videoid]--;
		LeaveCriticalSection(&VideoRecordRequestsCS[videoid]);
		bVideoRecording = FALSE;
		bDispRecordSymbol = FALSE;
	}
	if (bMissionRunning)
	{	
		//AbortMission();
		bDispPlaySymbol = FALSE;
		//if (bMissionPaused)
		//{	
		//	bDispPauseSymbol = FALSE;
		//}
	}
	if (windowname[0] != 0)
	{
		EnterCriticalSection(&OpenCVCS);
#ifndef USE_OPENCV_HIGHGUI_CPP_API
		cvDestroyWindow(windowname);
		cvNamedWindow(windowname, CV_WINDOW_AUTOSIZE);
		cvDestroyWindow(windowname);
#else
		cv::destroyWindow(windowname);
		cv::namedWindow(windowname, cv::WINDOW_AUTOSIZE);
		cv::destroyWindow(windowname);
#endif // !USE_OPENCV_HIGHGUI_CPP_API
		LeaveCriticalSection(&OpenCVCS);
		memset(windowname, 0, sizeof(windowname));
	}

	StopChronoQuick(&chrono_epu);
	StopChronoQuick(&chrono_alarms);
	StopChronoQuick(&chrono_pausing);
	StopChronoQuick(&chrono_playing);
	StopChronoQuick(&chrono_recording);

	if (!bExit) bExit = TRUE; // Unexpected program exit...

	return 0;
}
