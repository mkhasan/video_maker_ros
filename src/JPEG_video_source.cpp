/*
 * JPEGSource.cpp
 *
 *  Created on: Oct 18, 2019
 *      Author: usrc
 */


#include "ros/ros.h"

#include "video_maker/JPEG_video_source.h"
#include "client_interface/robot_exception.h"

#include <stdlib.h>
#include <fstream>
//#include <assert.h>
#include <cstdlib>

using namespace std;
namespace video_maker {

JPEGVideoSource::JPEGVideoSource(JPEGBuffer & frameBuffer, unsigned movieFPS)
	: fFrameBuffer(frameBuffer)
	, fMovieFPS(movieFPS)

{

	Reset();
}

JPEGVideoSource::~JPEGVideoSource() {
	ROS_DEBUG("In ~JPEGVideoSource()");
}

void JPEGVideoSource::Reset() {
	fSeqNo = 0;
	fSyncTimestamp = 0;
	fSyncTime.tv_sec = fSyncTime.tv_usec = 0;
}

const char * JPEGVideoSource::GetData(int &dataLen, struct timeval& resultPresentationTime, const void * ptr) {

	struct timeval timeNow;
	gettimeofday(&timeNow, NULL);
	if (fSyncTime.tv_sec == 0 && fSyncTime.tv_usec == 0) {
	    // This is the first timestamp that we've seen, so use the current
	    // 'wall clock' time as the synchronization time.  (This will be
	    // corrected later when we receive RTCP SRs.)
	    fSyncTimestamp = fTimestamp;
	    fSyncTime = timeNow;
	}

	int timestampDiff = fTimestamp - fSyncTimestamp;
	      // Note: This works even if the timestamp wraps around
	      // (as long as "int" is 32 bits)

	  // Divide this by the timestamp frequency to get real time:
	double timeDiff = timestampDiff/(double)FREQ;

	// Add this to the 'sync time' to get our result:
	unsigned const million = 1000000;
	unsigned seconds, uSeconds;
	if (timeDiff >= 0.0) {
		seconds = fSyncTime.tv_sec + (unsigned)(timeDiff);
		uSeconds = fSyncTime.tv_usec
		  + (unsigned)((timeDiff - (unsigned)timeDiff)*million);
		if (uSeconds >= million) {
			  uSeconds -= million;
			  ++seconds;
		}
	} else {
		timeDiff = -timeDiff;
		seconds = fSyncTime.tv_sec - (unsigned)(timeDiff);
		uSeconds = fSyncTime.tv_usec - (unsigned)((timeDiff - (unsigned)timeDiff)*million);

		if ((int)uSeconds < 0) {
			uSeconds += million;
			--seconds;
		}
	}
	resultPresentationTime.tv_sec = seconds;
	resultPresentationTime.tv_usec = uSeconds;

	const char * data = NULL;

	if (ptr == NULL) {
		char filename[100];
		sprintf(filename, "%s/sample_images/foobar%04d.jpg", std::getenv("HOME"),(fSeqNo)%500);

		//dataLen = JPEGVideoSource::ReadFile(filename, data, maxLen);
	}
	else {

		const shm_data * rawData = (const shm_data *) ptr;
		try {

			fFrameBuffer.Fill(rawData);
		}
		catch (const exception & e) {
			ROS_ERROR("Valid data not found, trying to get previous image");

		}

		data = fFrameBuffer.Data();
		dataLen = fFrameBuffer.Length();		// try to get previous data
		if (dataLen == 0)
			ROS_ERROR("Previous image not found");



	}

	//if (dataLen == 0)
		//THROW(RobotException, "Error in getting data");



	int stepInc = int((FREQ+EPSILON)/(double(fMovieFPS)));
	fTimestamp += stepInc;		// to be gene
	fSeqNo ++;

	return data;

}

int JPEGVideoSource::GetFileLength(const char * filename) {


	fstream file(filename, ios::in|ios::binary|ios::ate);

	if (file.is_open() == false)
		return -1;

	int size = file.tellg();

	file.close();
	return size;



}


int JPEGVideoSource::ReadFile(const char * filename, char * data, int maxLen) {


	fstream file(filename, ios::in|ios::binary|ios::ate);

	if (file.is_open()) {

		int size = file.tellg();

		if (size > maxLen) {
			file.close();
			return -1;
		}

		file.seekg(0, ios::beg);
		file.read(data, size);
		file.close();

		return size;
	}

	return -1;

}

}
