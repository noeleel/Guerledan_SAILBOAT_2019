// Prevent Visual Studio Intellisense from defining _WIN32 and _MSC_VER when we use 
// Visual Studio to edit Linux or Borland C++ code.
#ifdef __linux__
#	undef _WIN32
#endif // __linux__
#if defined(__GNUC__) || defined(__BORLANDC__)
#	undef _MSC_VER
#endif // defined(__GNUC__) || defined(__BORLANDC__)

#include "Ball.h"

// Need to be undefined at the end of the file...
// min and max might cause incompatibilities with GCC...
#ifndef _MSC_VER
#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif // !max
#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif // !min
#endif // !_MSC_VER

THREAD_PROC_RETURN_VALUE BallThread(void* pParam)
{
	UNREFERENCED_PARAMETER(pParam);

	// Missing error checking...
	int nbTotalPixels = videoimgwidth*videoimgheight;
	// Used to detect the bounds of the detected object.
	int* nbSelectedPixelsj = (int*)calloc(videoimgwidth, sizeof(int)); // Number of selected pixels for the column j.
	int* nbSelectedPixelsi = (int*)calloc(videoimgheight, sizeof(int)); // Number of selected pixels for the line i.
	// Number of selected pixels in the picture.
	int nbSelectedPixels = 0;
	// Number of selected pixels in the picture for the light.
	int nbSelectedPixelsLight_ball = 0;
	// Image indicating selected pixels.
	IplImage* SelectedPixelsImage = cvCreateImage(cvSize(videoimgwidth, videoimgheight), IPL_DEPTH_8U, 3);
	// Estimation of the center of the object in the picture.
	double obji = 0, objj = 0;
	// Estimation of the radius of the object (in pixels).
	int objRadius = 0;
	// Estimation of the bounding rectangle of the object (in pixels).
	double objBoundWidth = 0, objBoundHeight = 0;
	// Estimation of the orientation of the object (in rad).
	double objAngle = 0;
	BOOL bobjAngleValid = FALSE;
	// Estimated d to the object (in m).
	double objDistance = 0;
	// Estimated bearing to the object (in rad).
	double objBearing = 0;
	// Estimated elevation to the object (in rad).
	double objElevation = 0;
	// Real radius of the object (in m).
	//double objRealRadius = 0.15;
	// Parameters of the camera.
	double pixelAngleSize = M_PI*HorizontalBeam/(180.0*videoimgwidth); // Angular size of a pixel (in rad).
	// We only consider objects that covers that ratio of the picture (refers to the min size of the object on the picture).
	//double objPixRatio = 1.0/256.0;
	// We only consider objects that have an estimated radius (in pixels) greater than this one.
	int objMinRadius = 0;
	// We consider that we are on the detected object when the number of selected pixels for the 
	// line i or column j (in pixels) is greater than this one.
	int objMinnbSelectedPixelsij = 0;


	// Accuracy of the orientation of the object (in rad).
	double thetastep = M_PI/32.0;
	// Accuracy of the covering of the object for the computation of its orientation (w.r.t. object radius).
	double dstepobjRadiusratio = 1.0/10.0;
	// When this ratio of objRadius is outside the picture, the estimated orientation will not be considered as valid.
	double objRadiusouterrratio = 0.25;
	// When abs(1-objBounds/objRadius) < boundsradiuserrratio, the detected object has probably a square or circle 
	// shape and the estimated orientation will not be considered as valid.
	double boundsradiuserrratio = 0.25;	


	// Colors of the object (obtained using the Color Picker and Edit Colors tools in Microsoft Paint).
	//unsigned char rmin = 175, rmax = 255, gmin = 175, gmax = 255, bmin = 175, bmax = 255;
	//unsigned char hmin = 25, hmax = 65, smin = 45, smax = 240, lmin = 65, lmax = 210;
	//unsigned char hmin = 0, hmax = 25, smin = 75, smax = 240, lmin = 75, lmax = 210;
	//unsigned char hmin_invalid = 49, hmax_invalid = 124, smin_invalid = 48, smax_invalid = 73, lmin_invalid = 61, lmax_invalid = 146;
	//unsigned char hmin_invalid = 70, hmax_invalid = 140, smin = 120, smax = 240, lmin = 80, lmax = 160;
	//unsigned char hmin_invalid = 40, hmax_invalid = 140, smin = 100, smax = 240, lmin = 50, lmax = 190;

	//BOOL bTemporaryObjectDetected = FALSE;
	//BOOL bTemporaryObjectDetected_prev = bTemporaryObjectDetected;
	CHRONO chrono_mindetectionduration;

	char strtime_pic[MAX_BUF_LEN];
	char snapfilename[MAX_BUF_LEN];
	char picsnapfilename[MAX_BUF_LEN];
	char dtcfilename[MAX_BUF_LEN];
	char kmlfilename[MAX_BUF_LEN];
	FILE* kmlsnapfile = NULL;
	int pic_counter = 0;
	CHRONO chrono;

	// Missing error checking...
	IplImage* image = cvCreateImage(cvSize(videoimgwidth, videoimgheight), IPL_DEPTH_8U, 3);
	IplImage* overlayimage = cvCreateImage(cvSize(videoimgwidth, videoimgheight), IPL_DEPTH_8U, 3);
	cvSet(overlayimage, CV_RGB(0, 0, 0), NULL);
	int i = 0, j = 0, index = 0;

	CvFont font;
	char szText[256];
	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.0f, 1.0f);

	EnterCriticalSection(&strtimeCS);
	sprintf(logballtaskfilename, LOG_FOLDER"logballtask_%.64s.csv", strtime_fns());
	LeaveCriticalSection(&strtimeCS);
	logballtaskfile = fopen(logballtaskfilename, "w");
	if (logballtaskfile == NULL)
	{
		printf("Unable to create log file.\n");
		if (!bExit) bExit = TRUE; // Unexpected program exit...
		return 0;
	}

	fprintf(logballtaskfile, 
		"%% Time (in s); Distance to the ball (in m); Bearing to the ball (in rad); Elevation to the ball (in rad); "
		"Light status (1 : on, 0 : off); "
		"x ball position (in m); y ball position (in m); z ball position (in m); "
		"Latitude of the ball (in decimal degrees); Longitude of the ball (in decimal degrees); Altitude of the ball (in m);\n"
		); 
	fflush(logballtaskfile);

	StartChrono(&chrono);
	StartChrono(&chrono_mindetectionduration);

	for (;;)
	{
		mSleep(captureperiod);

		if (bExit) break;
		if ((!bBallDetection)&&(!bBallTrackingControl)) continue;

		cvSet(overlayimage, CV_RGB(0, 0, 0), NULL);

		EnterCriticalSection(&BallCS);
#pragma region Object detection
		// Initializations...
		memset(nbSelectedPixelsj, 0, videoimgwidth*sizeof(int));
		memset(nbSelectedPixelsi, 0, videoimgheight*sizeof(int));
		nbSelectedPixels = 0;
		nbSelectedPixelsLight_ball = 0;
		obji = 0;
		objj = 0;
		objMinRadius = (int)(videoimgwidth*objMinRadiusRatio_ball);
		objMinnbSelectedPixelsij = objMinRadius/2;

		// Get an image from the webcam or video.
		EnterCriticalSection(&imgsCS[videoid_ball]);
		cvCopy(imgs[videoid_ball], image, 0);
		LeaveCriticalSection(&imgsCS[videoid_ball]);

		// Convert image->imageData from char* to unsigned char* to work with color values in 0..255.
		unsigned char* data = reinterpret_cast<unsigned char*>(image->imageData);
		unsigned char* overlaydata = reinterpret_cast<unsigned char*>(overlayimage->imageData);

		// Correction of bad lines on the borders of the video...
		CorrectImageBordersRawBGR(data, videoimgwidth, videoimgheight, 2, 0, 0, 0);

#pragma region Simulated objects for tests
		//for (i = 0; i < image->height; i++)
		//{
		//	for (j = 0; j < image->width; j++)
		//	{
		//		int index = 3*(j+image->width*i);
		//		data[0+index] = 0;
		//		data[1+index] = 0;
		//		data[2+index] = 0;
		//	}
		//}

		//CvPoint points[1][20];
		////points[0][0] = cvPoint(20,40);
		////points[0][1] = cvPoint(40,20);
		////points[0][2] = cvPoint(140,120);
		////points[0][3] = cvPoint(120,140);
		//points[0][0] = cvPoint(40,20);
		//points[0][1] = cvPoint(20,40);
		//points[0][2] = cvPoint(120,140);
		//points[0][3] = cvPoint(140,120);
		////points[0][0] = cvPoint(100,10);
		////points[0][1] = cvPoint(100,200);
		////points[0][2] = cvPoint(150,200);
		////points[0][3] = cvPoint(150,10);
		////points[0][0] = cvPoint(10,100);
		////points[0][1] = cvPoint(200,100);
		////points[0][2] = cvPoint(200,150);
		////points[0][3] = cvPoint(10,150);

		//CvPoint* pts[1] = {points[0]};
		//int npts[] = {4};

		//if (((int)(10*GetTimeElapsedChronoQuick(&chrono)))%50 < 10) 
		//	if (((int)(10*GetTimeElapsedChronoQuick(&chrono)))%10 < 5) 
		//		cvFillPoly(image, pts, npts, 1, CV_RGB(255,0,0));
#pragma endregion

		for (i = 0; i < image->height; i++)
		{
			for (j = 0; j < image->width; j++)
			{
				index = 3*(j+image->width*i);
				double b = data[0+index];
				double g = data[1+index];
				double r = data[2+index];
				double h = 0, s = 0, l = 0;
				RGB2HSL_MSPaint(r, g, b, &h, &s, &l);
				// Select the pixels without the right color.
				//if (
				//	(
				//	(h < hmin_invalid)||(h > hmax_invalid)
				//	)&&(
				//	(s >= smin)&&(s <= smax)&&
				//	(l >= lmin)&&(l <= lmax)
				//	)
				//	)
				// Select the pixels with the right color.
				if (
					//((r >= rmin_ball)&&(r <= rmax_ball))&&
					//((g >= gmin_ball)&&(g <= gmax_ball))&&
					//((b >= bmin_ball)&&(b <= bmax_ball))
					//((h >= hmin_ball)&&(h <= hmax_ball))&&
					//((s >= smin_ball)&&(s <= smax_ball))&&
					//((l >= lmin_ball)&&(l <= lmax_ball))
					((h <= hmin_ball)||(h >= hmax_ball))&&
					((s >= smin_ball)&&(s <= smax_ball))&&
					((l >= lmin_ball)&&(l <= lmax_ball))
					)
				{
					SelectedPixelsImage->imageData[index] = 1;
					nbSelectedPixels++;


					nbSelectedPixelsi[i]++;
					nbSelectedPixelsj[j]++;

					//if (nbSelectedPixelsi[i] > 10) i0 = i;


					// Prepare the computation of the mean of selected pixels.
					obji += i;
					objj += j;
					// Selected pixels are displayed in red.
					overlaydata[0+index] = 0;
					overlaydata[1+index] = 0;
					overlaydata[2+index] = 255;
				}
				else
				{
					SelectedPixelsImage->imageData[index] = 0;
				}

				// Light detection.
				if ((r >= lightMin_ball)&&(g >= lightMin_ball)&&(b >= lightMin_ball))
				{
					nbSelectedPixelsLight_ball++;
					// Selected pixels for the light are displayed in green.
					overlaydata[0+index] = 0;
					overlaydata[1+index] = 255;
					overlaydata[2+index] = 0;
				}
			}
		}
#pragma endregion
		sprintf(szText, "DR=%.2f", detectratio_ball);
		cvPutText(overlayimage, szText, cvPoint(videoimgwidth-10*8,videoimgheight-40), &font, CV_RGB(255,0,128));
		if (nbSelectedPixels == 0) 
		{
			detectratio_ball = (0.001*(double)captureperiod/objDetectionRatioDuration_ball)*0.0+(1.0-(0.001*(double)captureperiod/objDetectionRatioDuration_ball))*detectratio_ball;
			//if (GetTimeElapsedChronoQuick(&chrono_mindetectionduration) > objDetectionRatioDuration_ball)
			//{
			//	detectratio_ball = min(0.0, detectratio_ball-(0.001*(double)captureperiod/objDetectionRatioDuration_ball));
			//}
			//bTemporaryObjectDetected = FALSE;
			LeaveCriticalSection(&BallCS);
			EnterCriticalSection(&BallOverlayImgCS);
			cvCopy(overlayimage, BallOverlayImg, 0);
			LeaveCriticalSection(&BallOverlayImgCS);
			if (bExit) break;
			continue;
		}

		// Compute an estimation of the radius of the detected object. 
		// sqrt() is used to virtually increase the radius because there are always missed selected pixels...
		objRadius = (int)(videoimgwidth*sqrt((double)nbSelectedPixels/(double)nbTotalPixels)/2.0);

#pragma region Bounding rectangle computation
		int i0 = -1, i1 = -1, j0 = -1, j1 = -1;
		// Get the first line that contains the detected object.
		for (i = 0; i < image->height; i++)
		{
			if (nbSelectedPixelsi[i] > objMinnbSelectedPixelsij) 
			{
				i0 = i;
				break;
			}
		}
		// Get the last line that contains the detected object.
		for (i = image->height-1; i >= 0; i--)
		{
			if (nbSelectedPixelsi[i] > objMinnbSelectedPixelsij) 
			{
				i1 = i;
				break;
			}
		}
		// Get the first column that contains the detected object.
		for (j = 0; j < image->width; j++)
		{
			if (nbSelectedPixelsj[j] > objMinnbSelectedPixelsij) 
			{
				j0 = j;
				break;
			}
		}
		// Get the last column that contains the detected object.
		for (j = image->width-1; j >= 0; j--)
		{
			if (nbSelectedPixelsj[j] > objMinnbSelectedPixelsij) 
			{
				j1 = j;
				break;
			}
		}

		objBoundWidth = j1-j0;
		objBoundHeight = i1-i0;

		cvRectangle(overlayimage, cvPoint(j0,i0), cvPoint(j1,i1), CV_RGB(128,0,255));
#pragma endregion
		if (objRadius > objMinRadius)
		{
#pragma region Object characteristics computations
			// Compute the mean of selected pixels.
			obji = obji/(double)nbSelectedPixels;
			objj = objj/(double)nbSelectedPixels;

			//cvPutText(overlayimage, "Object detected", cvPoint(10,20), &font, CV_RGB(0,255,0));
			cvRectangle(overlayimage, 
				cvPoint((int)objj-objRadius,(int)obji-objRadius), 
				cvPoint((int)objj+objRadius,(int)obji+objRadius), 
				CV_RGB(0,255,0));

			objDistance = objRealRadius_ball/tan(objRadius*pixelAngleSize);
			objBearing = -(objj-image->width/2.0)*pixelAngleSize;
			objElevation = -(obji-image->height/2.0)*pixelAngleSize;

#define THIRD_METHOD
#ifdef FIRST_METHOD
			objAngle = (3.14/4.0)*floor(0.5+atan(objBoundHeight/objBoundWidth)/(3.14/4.0));

			cvLine(overlayimage, 
				cvPoint((int)(objj-objRadius*cos(objAngle)),(int)(obji+objRadius*sin(objAngle))), 
				cvPoint((int)(objj+objRadius*cos(objAngle)),(int)(obji-objRadius*sin(objAngle))), 
				CV_RGB(0,255,0));

			cvLine(overlayimage, 
				cvPoint((int)(objj-objRadius*cos(objAngle+3.14/2.0)),(int)(obji+objRadius*sin(objAngle+3.14/2.0))), 
				cvPoint((int)(objj+objRadius*cos(objAngle+3.14/2.0)),(int)(obji-objRadius*sin(objAngle+3.14/2.0))), 
				CV_RGB(0,255,0));
#endif // FIRST_METHOD

#ifdef SECOND_METHOD
			objAngle = min(
				max(
				0.0,
				(1.0-pow(min((double)nbSelectedPixels/(double)(objBoundWidth*objBoundHeight),1.0), 4))*3.14/4.0
				), 
				3.14/4.0
				);

			//printf("objBoundWidth = %f, objBoundHeight = %f, ratio = %f, angle = %f\n", objBoundWidth, objBoundHeight, (double)nbSelectedPixels/(double)(objBoundWidth*objBoundHeight), objAngle);

			cvLine(overlayimage, 
				cvPoint((int)(objj-objRadius*cos(objAngle)),(int)(obji+objRadius*sin(objAngle))), 
				cvPoint((int)(objj+objRadius*cos(objAngle)),(int)(obji-objRadius*sin(objAngle))), 
				CV_RGB(0,255,0));

			cvLine(overlayimage, 
				cvPoint((int)(objj-objRadius*cos(objAngle+3.14/2.0)),(int)(obji+objRadius*sin(objAngle+3.14/2.0))), 
				cvPoint((int)(objj+objRadius*cos(objAngle+3.14/2.0)),(int)(obji-objRadius*sin(objAngle+3.14/2.0))), 
				CV_RGB(0,255,0));
#endif // SECOND_METHOD

#ifdef THIRD_METHOD

			// Compute the mean angle where there is the most of selected pixels for the estimation 
			// of the detected object orientation...

			double theta = 0, d = 0, cosmean = 0, sinmean = 0;
			double dstep = objRadius*dstepobjRadiusratio;
			//int nbSelectedPixelstheta = 0;

			for (theta = -M_PI/2.0; theta < M_PI/2.0; theta += thetastep)
			{
				for (d = 0.0; d < objBoundWidth+objBoundHeight; d += dstep)
				{
					double costheta = cos(theta);
					double sintheta = sin(theta);
					i = (int)(obji-d*sintheta);
					j = (int)(objj+d*costheta);
					index = 3*(j+image->width*i);
					if ((index >= 0)&&(index < 3*nbTotalPixels-3))
					{
						//unsigned char b = data[0+index];
						//unsigned char g = data[1+index];
						//unsigned char r = data[2+index];

						//// Check if the pixel was selected.
						//if ((r == 255)&&(g == 0)&&(b == 0))
						if (SelectedPixelsImage->imageData[index] == 1)
						{
							//nbSelectedPixelstheta++;
							// *2.0 because of periodicity problems...
							cosmean += cos(theta*2.0);
							sinmean += sin(theta*2.0);
						}
					}
				}
			}

			// Mean of cos and sin... not needed in this case...
			//cosmean = cosmean/(double)nbSelectedPixelstheta;
			//sinmean = sinmean/(double)nbSelectedPixelstheta;

			// /2.0 because previously *2.0...
			objAngle = atan2(sinmean, cosmean)/2.0;

			//printf("angle = %f degrees\n", objAngle*180/3.14);

			// Validity of the orientation angle depends on bounds/radius ratio 
			// and on whether the objRadius square is inside the image or not...

			if (
				(objj-objRadius >= -objRadiusouterrratio*objRadius)&&(obji-objRadius >= -objRadiusouterrratio*objRadius)&&
				(objj+objRadius < videoimgwidth+objRadiusouterrratio*objRadius)&&(obji+objRadius < videoimgheight+objRadiusouterrratio*objRadius)
				&&
				(
				(objBoundWidth/(2.0*objRadius) < 1.0-boundsradiuserrratio)||(objBoundWidth/(2.0*objRadius) > 1.0+boundsradiuserrratio)||
				(objBoundHeight/(2.0*objRadius) < 1.0-boundsradiuserrratio)||(objBoundHeight/(2.0*objRadius) > 1.0+boundsradiuserrratio)
				)
				)
			{
				bobjAngleValid = TRUE;
				cvLine(overlayimage, 
					cvPoint((int)(objj-objRadius*cos(objAngle)),(int)(obji+objRadius*sin(objAngle))), 
					cvPoint((int)(objj+objRadius*cos(objAngle)),(int)(obji-objRadius*sin(objAngle))), 
					CV_RGB(0,255,0));
			}
			else
			{
				bobjAngleValid = FALSE;
				cvLine(overlayimage, 
					cvPoint((int)(objj-objRadius*cos(objAngle)),(int)(obji+objRadius*sin(objAngle))), 
					cvPoint((int)(objj+objRadius*cos(objAngle)),(int)(obji-objRadius*sin(objAngle))), 
					CV_RGB(255,0,255));
			}					
#endif // THIRD_METHOD
#pragma endregion
			detectratio_ball = (0.001*(double)captureperiod/objDetectionRatioDuration_ball)*1.0+(1.0-(0.001*(double)captureperiod/objDetectionRatioDuration_ball))*detectratio_ball;
			//if (fabs(detectratio_ball) < 0.001)
			//{
			//	StopChronoQuick(&chrono_mindetectionduration);
			//	StartChrono(&chrono_mindetectionduration);
			//}
			//detectratio_ball = max(1.0, detectratio_ball+(0.001*(double)captureperiod/objDetectionRatioDuration_ball));
			//bTemporaryObjectDetected = TRUE;
		}
		else
		{
			detectratio_ball = (0.001*(double)captureperiod/objDetectionRatioDuration_ball)*0.0+(1.0-(0.001*(double)captureperiod/objDetectionRatioDuration_ball))*detectratio_ball;
			//if (GetTimeElapsedChronoQuick(&chrono_mindetectionduration) > objDetectionRatioDuration_ball)
			//{
			//	detectratio_ball = min(0.0, detectratio_ball-(0.001*(double)captureperiod/objDetectionRatioDuration_ball));
			//}
			//bTemporaryObjectDetected = FALSE;
		}

		/*
		{
			nbdetect++;
			detectratio_ball = nbdetect/(nbdetect+nbnondetect);
			if (nbdetect+nbnondetect > objDetectionRatioDuration_ball/(0.001*(double)captureperiod)+1)
			{
				nbdetect--;
				nbnondetect--;
			}
		}
		else
		{
			nbnondetect++;
			detectratio_ball = nbdetect/(nbdetect+nbnondetect);
			if (nbdetect+nbnondetect > objDetectionRatioDuration_ball/(0.001*(double)captureperiod)+1)
			{
				nbdetect--;
				nbnondetect--;
			}
		}		
		*/

		if (detectratio_ball >= objMinDetectionRatio_ball)
		{
			bBallFound = TRUE;
#pragma region Actions
			sprintf(szText, "RNG=%.2fm,BRG=%ddeg,ELV=%ddeg", objDistance, (int)(objBearing*180.0/M_PI), (int)(objElevation*180.0/M_PI));
			cvPutText(overlayimage, szText, cvPoint(10,videoimgheight-20), &font, CV_RGB(255,0,128));

			if (nbSelectedPixelsLight_ball > nbTotalPixels*lightPixRatio_ball) 
			{
				lightStatus_ball = 1; 
				cvPutText(overlayimage, "Light", cvPoint(10,videoimgheight-40), &font, CV_RGB(0,255,0));
			}
			else 
			{
				lightStatus_ball = 0;
			}

			// Send acoustic message when ball detected...
			if (bAcoustic_ball)
			{
				EnterCriticalSection(&MDMCS);
				AcousticCommandMDM = SENDOPI_MSG;
				LeaveCriticalSection(&MDMCS);
			}
			else
			{
				//// Temp...
				//EnterCriticalSection(&MDMCS);
				//AcousticCommandMDM = 0;
				//LeaveCriticalSection(&MDMCS);
			}

			EnterCriticalSection(&StateVariablesCS);
			switch (camdir_ball)
			{
			case 0:
				x_ball = Center(xhat)+objDistance*cos(Center(psihat)+objBearing); 
				y_ball = Center(yhat)+objDistance*sin(Center(psihat)+objBearing); 
				z_ball = Center(zhat)+objDistance*sin(objElevation);
				break;
			case 1:
				x_ball = Center(xhat)+objDistance*cos(Center(psihat)+objBearing+M_PI/2.0); 
				y_ball = Center(yhat)+objDistance*sin(Center(psihat)+objBearing+M_PI/2.0); 
				z_ball = Center(zhat)+objDistance*sin(objElevation);
				break;
			case -1:
				x_ball = Center(xhat)+objDistance*cos(Center(psihat)+objBearing-M_PI/2.0); 
				y_ball = Center(yhat)+objDistance*sin(Center(psihat)+objBearing-M_PI/2.0); 
				z_ball = Center(zhat)+objDistance*sin(objElevation);
				break;
			case 2:
			default:
				x_ball = Center(xhat); 
				y_ball = Center(yhat); 
				z_ball = Center(zhat)-objDistance;
				break;
			}
			EnvCoordSystem2GPS(lat_env, long_env, alt_env, angle_env, x_ball, y_ball, z_ball, &lat_ball, &long_ball, &alt_ball);
			LeaveCriticalSection(&StateVariablesCS);

			fprintf(logballtaskfile, "%f;%f;%f;%f;%d;%f;%f;%f;%f;%f;%f;\n", 
				GetTimeElapsedChronoQuick(&chrono), objDistance, objBearing, objElevation, 
				lightStatus_ball, x_ball, y_ball, z_ball, lat_ball, long_ball, alt_ball
				);
			fflush(logballtaskfile);

			if (bBallDetection)
			{
				// Save a picture showing the detection.
				memset(strtime_pic, 0, sizeof(strtime_pic));
				EnterCriticalSection(&strtimeCS);
				strcpy(strtime_pic, strtime_fns());
				LeaveCriticalSection(&strtimeCS);
				sprintf(snapfilename, "pic_%.64s.jpg", strtime_pic);
				sprintf(picsnapfilename, PIC_FOLDER"pic_%.64s.jpg", strtime_pic);
				if (!cvSaveImage(picsnapfilename, image, 0))
				{
					printf("Error saving a picture file.\n");
				}
				sprintf(dtcfilename, PIC_FOLDER"pic_%.64s.png", strtime_pic);
				if (!cvSaveImage(dtcfilename, overlayimage, 0))
				{
					printf("Error saving a picture file.\n");
				}
				sprintf(kmlfilename, PIC_FOLDER"pic_%.64s.kml", strtime_pic);
				kmlsnapfile = fopen(kmlfilename, "w");
				if (kmlsnapfile != NULL)
				{
					EnterCriticalSection(&StateVariablesCS);
					fprintf(kmlsnapfile, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
					fprintf(kmlsnapfile, "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">\n");
					fprintf(kmlsnapfile, "<Document>\n<name>pic_%.64s</name>\n", strtime_pic);
					fprintf(kmlsnapfile, "\t<PhotoOverlay>\n\t\t<name>pic_%.64s</name>\n", strtime_pic);
					fprintf(kmlsnapfile, "\t\t<Camera>\n\t\t\t<longitude>%.8f</longitude>\n\t\t\t<latitude>%.8f</latitude>\n\t\t\t<altitude>%.3f</altitude>\n", long_ball, lat_ball, alt_ball);
					fprintf(kmlsnapfile, "\t\t\t<heading>%f</heading>\n\t\t\t<tilt>%f</tilt>\n\t\t\t<roll>%f</roll>\n", (fmod_2PI(-angle_env-Center(psihat)+3.0*M_PI/2.0)+M_PI)*180.0/M_PI, 0.0, 0.0);
					fprintf(kmlsnapfile, "\t\t\t<altitudeMode>relativeToGround</altitudeMode>\n\t\t\t<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode>\n\t\t</Camera>\n");
					fprintf(kmlsnapfile, "\t\t<Style>\n\t\t\t<IconStyle>\n\t\t\t\t<Icon>\n\t\t\t\t\t<href>:/camera_mode.png</href>\n\t\t\t\t</Icon>\n\t\t\t</IconStyle>\n");
					fprintf(kmlsnapfile, "\t\t\t<ListStyle>\n\t\t\t\t<listItemType>check</listItemType>\n\t\t\t\t<ItemIcon>\n\t\t\t\t\t<state>open closed error fetching0 fetching1 fetching2</state>\n");
					fprintf(kmlsnapfile, "\t\t\t\t\t<href>http://maps.google.com/mapfiles/kml/shapes/camera-lv.png</href>\n\t\t\t\t</ItemIcon>\n\t\t\t\t<bgColor>00ffffff</bgColor>\n\t\t\t\t<maxSnippetLines>2</maxSnippetLines>\n");
					fprintf(kmlsnapfile, "\t\t\t</ListStyle>\n\t\t</Style>\n");
					fprintf(kmlsnapfile, "\t\t<Icon>\n\t\t\t<href>%.255s</href>\n\t\t</Icon>\n", snapfilename);
					fprintf(kmlsnapfile, "\t\t<ViewVolume>\n\t\t\t<leftFov>-25</leftFov>\n\t\t\t<rightFov>25</rightFov>\n\t\t\t<bottomFov>-16.25</bottomFov>\n\t\t\t<topFov>16.25</topFov>\n\t\t\t<near>7.92675</near>\n\t\t</ViewVolume>\n");
					fprintf(kmlsnapfile, "\t\t<Point>\n\t\t\t<altitudeMode>relativeToGround</altitudeMode>\n\t\t\t<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode>\n\t\t\t<coordinates>%.8f,%.8f,%.3f</coordinates>\n\t\t</Point>\n", long_ball, lat_ball, alt_ball);
					fprintf(kmlsnapfile, "\t</PhotoOverlay>\n");
					fprintf(kmlsnapfile, "</Document>\n</kml>\n");
					LeaveCriticalSection(&StateVariablesCS);
					fclose(kmlsnapfile);
				}
				else
				{
					printf("Error saving a KML file.\n");
				}

				if (bBrake_ball)
				{
					// Temporary...
					EnterCriticalSection(&StateVariablesCS);
					u = 0;
					bDistanceControl = FALSE;
					bBrakeControl = TRUE;
					LeaveCriticalSection(&StateVariablesCS);
					mSleep(3000);
					EnterCriticalSection(&StateVariablesCS);
					u = 0;
					bBrakeControl = FALSE;
					LeaveCriticalSection(&StateVariablesCS);
				}
				bBallDetection = FALSE;
			}

			if (bBallTrackingControl)
			{
				if (pic_counter > (int)(1000/captureperiod))
				{
					pic_counter = 0;
					// Save a picture showing the detection.
					memset(strtime_pic, 0, sizeof(strtime_pic));
					EnterCriticalSection(&strtimeCS);
					strcpy(strtime_pic, strtime_fns());
					LeaveCriticalSection(&strtimeCS);
					sprintf(snapfilename, "pic_%.64s.jpg", strtime_pic);
					sprintf(picsnapfilename, PIC_FOLDER"pic_%.64s.jpg", strtime_pic);
					if (!cvSaveImage(picsnapfilename, image, 0))
					{
						printf("Error saving a picture file.\n");
					}
					sprintf(dtcfilename, PIC_FOLDER"pic_%.64s.png", strtime_pic);
					if (!cvSaveImage(dtcfilename, overlayimage, 0))
					{
						printf("Error saving a picture file.\n");
					}
					sprintf(kmlfilename, PIC_FOLDER"pic_%.64s.kml", strtime_pic);
					kmlsnapfile = fopen(kmlfilename, "w");
					if (kmlsnapfile != NULL)
					{
						EnterCriticalSection(&StateVariablesCS);
						fprintf(kmlsnapfile, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
						fprintf(kmlsnapfile, "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">\n");
						fprintf(kmlsnapfile, "<Document>\n<name>pic_%.64s</name>\n", strtime_pic);
						fprintf(kmlsnapfile, "\t<PhotoOverlay>\n\t\t<name>pic_%.64s</name>\n", strtime_pic);
						fprintf(kmlsnapfile, "\t\t<Camera>\n\t\t\t<longitude>%.8f</longitude>\n\t\t\t<latitude>%.8f</latitude>\n\t\t\t<altitude>%.3f</altitude>\n", long_ball, lat_ball, alt_ball);
						fprintf(kmlsnapfile, "\t\t\t<heading>%f</heading>\n\t\t\t<tilt>%f</tilt>\n\t\t\t<roll>%f</roll>\n", (fmod_2PI(-angle_env-Center(psihat)+3.0*M_PI/2.0)+M_PI)*180.0/M_PI, 0.0, 0.0);
						fprintf(kmlsnapfile, "\t\t\t<altitudeMode>relativeToGround</altitudeMode>\n\t\t\t<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode>\n\t\t</Camera>\n");
						fprintf(kmlsnapfile, "\t\t<Style>\n\t\t\t<IconStyle>\n\t\t\t\t<Icon>\n\t\t\t\t\t<href>:/camera_mode.png</href>\n\t\t\t\t</Icon>\n\t\t\t</IconStyle>\n");
						fprintf(kmlsnapfile, "\t\t\t<ListStyle>\n\t\t\t\t<listItemType>check</listItemType>\n\t\t\t\t<ItemIcon>\n\t\t\t\t\t<state>open closed error fetching0 fetching1 fetching2</state>\n");
						fprintf(kmlsnapfile, "\t\t\t\t\t<href>http://maps.google.com/mapfiles/kml/shapes/camera-lv.png</href>\n\t\t\t\t</ItemIcon>\n\t\t\t\t<bgColor>00ffffff</bgColor>\n\t\t\t\t<maxSnippetLines>2</maxSnippetLines>\n");
						fprintf(kmlsnapfile, "\t\t\t</ListStyle>\n\t\t</Style>\n");
						fprintf(kmlsnapfile, "\t\t<Icon>\n\t\t\t<href>%.255s</href>\n\t\t</Icon>\n", snapfilename);
						fprintf(kmlsnapfile, "\t\t<ViewVolume>\n\t\t\t<leftFov>-25</leftFov>\n\t\t\t<rightFov>25</rightFov>\n\t\t\t<bottomFov>-16.25</bottomFov>\n\t\t\t<topFov>16.25</topFov>\n\t\t\t<near>7.92675</near>\n\t\t</ViewVolume>\n");
						fprintf(kmlsnapfile, "\t\t<Point>\n\t\t\t<altitudeMode>relativeToGround</altitudeMode>\n\t\t\t<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode>\n\t\t\t<coordinates>%.8f,%.8f,%.3f</coordinates>\n\t\t</Point>\n", long_ball, lat_ball, alt_ball);
						fprintf(kmlsnapfile, "\t</PhotoOverlay>\n");
						fprintf(kmlsnapfile, "</Document>\n</kml>\n");
						LeaveCriticalSection(&StateVariablesCS);
						fclose(kmlsnapfile);
					}
					else
					{
						printf("Error saving a KML file.\n");
					}
				}
				else pic_counter++;

				if (bBrake_ball)
				{
					// No need to really brake...
				}
				else
				{
					// Tracking.
					switch (camdir_ball)
					{
					case 0:
						EnterCriticalSection(&StateVariablesCS);
						/*
							objDistance_ball = objDistance;
							wpsi_ball = Center(psihat) + objBearing;
						*/
						dist = objDistance;
						wd = d0_ball;
						wu = u_ball;
						wpsi = Center(psihat)+objBearing;
						if (bDepth_ball) wz = Center(zhat)+objDistance*sin(objElevation);
						bDistanceControl = TRUE;
						bHeadingControl = TRUE;
						if (bDepth_ball) 
						{
							bDepthControl = TRUE;
							bAltitudeAGLControl = FALSE;
						}
						LeaveCriticalSection(&StateVariablesCS);
						break;
					case 1:
						EnterCriticalSection(&StateVariablesCS);
						u = u_ball;
						wpsi = Center(psihat)+objBearing+u_ball*(M_PI/4.0)*sign(objDistance-d0_ball, wdradius);
						if (bDepth_ball) wz = Center(zhat)+objDistance*sin(objElevation);
						bHeadingControl = TRUE;
						if (bDepth_ball) 
						{
							bDepthControl = TRUE;
							bAltitudeAGLControl = FALSE;
						}
						LeaveCriticalSection(&StateVariablesCS);
						break;
					case -1:
						EnterCriticalSection(&StateVariablesCS);
						u = u_ball;
						wpsi = Center(psihat)+objBearing-u_ball*(M_PI/4.0)*sign(objDistance-d0_ball, wdradius);
						if (bDepth_ball) wz = Center(zhat)+objDistance*sin(objElevation);
						bHeadingControl = TRUE;
						if (bDepth_ball) 
						{
							bDepthControl = TRUE;
							bAltitudeAGLControl = FALSE;
						}
						LeaveCriticalSection(&StateVariablesCS);
						break;
					case 2:
						EnterCriticalSection(&StateVariablesCS);
						dist = objDistance*sin(objElevation);
						wd = 0;
						wu = u_ball;
						wpsi = Center(psihat)+objBearing;
						if (bDepth_ball) wz = Center(zhat)-objDistance+d0_ball;
						bDistanceControl = TRUE;
						bHeadingControl = TRUE;
						if (bDepth_ball) 
						{
							bDepthControl = TRUE;
							bAltitudeAGLControl = FALSE;
						}
						LeaveCriticalSection(&StateVariablesCS);
						break;
					default:
						break;
					}
				}
			}

			if (procid_ball != -1)
			{
				if (bBallTrackingControl)
				{
					// stopballtracking to avoid multiple execute...
					bBallTrackingControl = FALSE;
					bDistanceControl = FALSE;
					//if (bBrake_ball) bBrakeControl = FALSE;
					bHeadingControl = FALSE;
					if (bDepth_ball) 
					{
						bDepthControl = FALSE;
						bAltitudeAGLControl = FALSE;
					}
				}
				if (bEcho) printf("execute %d\n", procid_ball);
				ExecuteProcedure(procid_ball);
				bWaiting = FALSE; // To interrupt and force execution of the next commands...
			}
#pragma endregion
		}
		else
		{
			pic_counter = 1000; // To force to save the first object image upon detection...
		}

		LeaveCriticalSection(&BallCS);

		EnterCriticalSection(&BallOverlayImgCS);
		cvCopy(overlayimage, BallOverlayImg, 0);
		LeaveCriticalSection(&BallOverlayImgCS);

		if (bExit) break;
	}

	StopChronoQuick(&chrono_mindetectionduration);
	StopChronoQuick(&chrono);

	fclose(logballtaskfile);

	cvReleaseImage(&overlayimage);
	cvReleaseImage(&image);

	free(nbSelectedPixelsi);
	free(nbSelectedPixelsj);

	cvReleaseImage(&SelectedPixelsImage);

	if (!bExit) bExit = TRUE; // Unexpected program exit...

	return 0;
}

// min and max might cause incompatibilities with GCC...
#ifndef _MSC_VER
#ifdef max
#undef max
#endif // max
#ifdef min
#undef min
#endif // min
#endif // !_MSC_VER
