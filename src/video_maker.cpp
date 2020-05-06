/*
 * video_maker.cpp
 *
 *  Created on: Oct 22, 2019
 *      Author: kict
 */

#include "ros/ros.h"

#include "video_maker/video_maker.h"
#include "video_maker/JPEG_buffer.h"
#include "video_maker/JPEG_video_source.h"



#include "client_interface/robot_exception.h"


//


#include <ace/Task.h>
#include <exception>
#include <cstdlib>
#include <sstream>
#include <string>


using namespace std;

using namespace client_interface;


namespace video_maker {

static shm_data localData[ShmManager::MAX_NUM_CAMERA];
static FrameBuffer buffer[ShmManager::MAX_NUM_CAMERA];
typedef enum {IDLE, CREATING, UPDATING, CLOSING, TEST} state_t;



}



class Dim {
public:
	Dim(unsigned _width, unsigned _height): width(_width), height(_height) {

	}
	unsigned Width() const {
		return width;
	}
	unsigned Height() const {
		return height;
	}
private:
	unsigned width;
	unsigned height;

};


void * video_maker::video_maker_func(void * ptr) {

	video_maker::info_t *pInfo = (video_maker::info_t *) ptr;
	bool &quit = pInfo->quit;

	const int epsilon = 0.001;
	unsigned int periodUs = (unsigned int) ((1000000.0/FRAME_RATE) + epsilon);

	printf("period is %d \n", periodUs);
	int steamID = 1;





	if (pInfo->cameraNames.size() == 0) {
		ROS_ERROR("No camera found");
		return NULL;
	}

	int size = pInfo->cameraNames.size();

	vector<Source *> sources;
	vector<FILE *> files;


	Dim dim0(640, 320);
	Dim dim1(1920, 1080);

	for (int k=0; k<size; ++k) {

		JPEGVideoSource *ptr = new JPEGVideoSource(buffer[k], FRAME_RATE);
		sources.push_back((Source *)ptr);

		Dim & dim = (k == 0 ? dim0 : dim1);

		pInfo->sinks.push_back(new AVIFileSink(ptr, buffer[k].GetBufferSize(), dim.Width(), dim.Height(), FRAME_RATE));

		files.push_back(NULL);
	}

	vector<string> prefix;
	for (int i=0; i<pInfo->cameraNames.size(); ++i) {
		string str = pInfo->cameraNames[i]+string("_");
		prefix.push_back(str);
		ROS_DEBUG(str.c_str());
	}


	try {
		ShmInitialize(*pInfo);
	}
	catch (const exception & e) {
		ROS_ERROR("Error: %s", e.what());

		return NULL;
	}

	state_t state = IDLE;

	ACE_Time_Value twakeup, tinc, tsleep, t1;
	tinc.set(0, periodUs);
	int count = 0;
	twakeup = ACE_OS::gettimeofday();
	int noFrames = -1;
	const unsigned minFrames = MIN_CLIP_DURATION*FRAME_RATE;
	const unsigned maxFrames = MAX_CLIP_DURATION*FRAME_RATE;
	while(quit == false) {

		twakeup += tinc;

		try {

			if (state == IDLE && pInfo->mode != info_t::NONE )
				state = CREATING;

			if(state == CREATING) {
				CreateFiles(prefix, files, pInfo->mode, pInfo->video_root);
				for (int k=0; k<size; ++k)
					pInfo->sinks[k]->Initialize(files[k]);
				state = UPDATING;
				noFrames = -1;
			}

			if(state == UPDATING) {

				//ROS_DEBUG("going to update %s", PrintDateTime());

				if (noFrames >= 0)
					Update(*pInfo);
				//ROS_DEBUG("update done %s", PrintDateTime());
				noFrames ++;
				if (noFrames >= maxFrames && pInfo->mode != info_t::NONE) {
					state = CLOSING;
					//pInfo->mode = info_t::NONE;
				}
				if(pInfo->mode == info_t::NONE) {
					if (noFrames >= minFrames)
						state = CLOSING;
				}

			}

			if(state == CLOSING) {
				for (int k=0; k<size; ++k) {
					pInfo->sinks[k]->Finalize();
					files[k] = NULL;
				}
				ROS_DEBUG("cloisng done");
				state = IDLE;
			}


		}
		catch(const exception & e) {
			 ROS_ERROR("Error: %s", e.what());
		}

		t1 = ACE_OS::gettimeofday();
		tsleep= twakeup - t1;
		 //  cout<< "tsleep : "<<tsleep<<" twakeup : "<<twakeup<<" t1 :"<<t1<<endl;
		if (tsleep > ACE_Time_Value::zero)
				   ACE_OS::sleep(tsleep);

		else {
			ROS_INFO("No sleep %d", noFrames);
		}

		if (count == 10 && state == IDLE) {
			state = CREATING;

		}
		else if(count == 200+10 && state != IDLE) {
			state = CLOSING;
		}

		//count ++;

	}



	Source * source;
	AVIFileSink *sink;
	for (int k=0; k<size; ++k) {

		sink = pInfo->sinks[k];
		delete sink;

		source = sources[k];
		delete source;
	}

	ShmFinalize(*pInfo);




}


void video_maker::ShmInitialize(info_t & info) {

	int size = info.cameraNames.size();

	if (size <= 0 || size > ShmManager::MAX_NUM_CAMERA) {
		THROW(RobotException, "Invaid size paramter");
	}

	char *shm[ShmManager::MAX_NUM_CAMERA];

	for (int k=0; k<ShmManager::MAX_NUM_CAMERA; ++k) {
		info.pData[k] = NULL;
	}

	for (int k=0; k < size; ++k) {
		string mutexName;
		stringstream ss;
		ss << k;


		mutexName = info.mutexPrefix+ss.str();
		info.mutexList[k] = sem_open(mutexName.c_str(), O_RDWR);
		if (info.mutexList[k] == SEM_FAILED) {

			char msg[256];
			sprintf(msg, "Error in mutex initialize of id %d (mutexName:%s)", k, mutexName.c_str());
			ShmFinalize(info, k);
			THROW(RobotException, msg);
		}


	    info.shm_client[k] = ACE_Shared_Memory_SV(ShmManager::SHM_KEY_START+k, sizeof (shm_data));

	    shm[k] = (char *) info.shm_client[k].malloc ();

	    if (shm == NULL) {
			char msg[256];
			sprintf(msg, "Error in shm memory Initialize of id %d", k);
	    	ShmFinalize(info,size);
	    	for (int i=0; i<k; i++)
	    		info.shm_client[i].remove();
	    	THROW(RobotException, msg);
	    }



	}

	for(int k=0; k<size; ++k)
		info.pData[k] = new (shm[k]) shm_data;


}

void video_maker::ShmFinalize(info_t & info) {
	int size = info.cameraNames.size();
	ShmFinalize(info, size);
}

void video_maker::ShmFinalize(info_t & info, int n) {

	for (int k=0; k<n; k++) {
		//string mutexName;
		//stringstream ss;
		//ss << k;

		sem_close(info.mutexList[k]);

		ROS_DEBUG("Finalizing camera id:%d", k);
	}
}

void video_maker::CreateFiles(const vector<string> & prefix, vector<FILE *> & files, info_t::mode_t curMode, const string & path) {

	char dateTimeStr[256];
	PrintDateTime(dateTimeStr);
	if(prefix.size() != files.size())
		THROW(RobotException, "prefix size and files size must be equal");


	if(curMode != info_t::AUTO && curMode != info_t::SNAP_AROUND)
		THROW(RobotException, "improper mode");

	string filename;
	string subdir = curMode == info_t::AUTO ? "auto/" : "move_around/";


	for (int i=0; i<files.size(); i++) {
		//sprintf(fileName, "%s_%s.avi", prefix[i], dateTimeStr);
		filename = path + subdir + prefix[i] + string(dateTimeStr) + string(".avi");
		files[i] = fopen(filename.c_str(), "wb");
		if (files[i] ==  NULL) {
			char msg[256];
			sprintf(msg, "Error in creating file %s", filename.c_str());
			THROW(RobotException, msg);
		}
		ROS_DEBUG("file %d: %s created", i, filename.c_str());

	}

}

void video_maker::CloseFiles(vector<FILE *> & files) {
	for(int k = 0; k<files.size(); ++k) {
		if (files[k] != NULL) {
			fclose(files[k]);
			files[k] = NULL;
		}
	}
}

char * video_maker::PrintDateTime() {
	static char buf[256];
	PrintDateTime(buf);
	return buf;
}

void video_maker::PrintDateTime(char *str) {
	struct timeval tv;
	struct timezone tz;
	struct tm *tm;
	gettimeofday(&tv, &tz);
	tm=localtime(&tv.tv_sec);
	sprintf(str, "%d-%d-%d_%d-%02d-%02d-%d", tm->tm_year+1900, tm->tm_mon, tm->tm_mday, tm->tm_hour, tm->tm_min,
		  tm->tm_sec, tv.tv_usec);
}

void video_maker::UpdateLocalData(info_t & info) {
	int size = info.cameraNames.size();
	for (int k=0; k<size; k++) {
		if(sem_wait(info.mutexList[k])< 0) {
			string msg = "Error in getting data from cameara ";
			msg += info.cameraNames[k];
			ROS_ERROR(msg.c_str());
			continue;
		}

		localData[k] = *(info.pData[k]);

		sem_post(info.mutexList[k]);


	}
}



void video_maker::Update(info_t & info) {


	UpdateLocalData(info);

	for (int k=0; k<info.sinks.size(); ++k) {
		ROS_DEBUG("Adding frame for camera %s", info.cameraNames[k].c_str());
		info.sinks[k]->AddNextFrame(localData[k]);
	}
	/*
	int size = info.files.size();
	for (int k=0; k<size; ++k) {

		buffer[k].Fill(&localData[k]);

	}

	for (int k=0; k<size; ++k) {
		if(info.files[k] != NULL) {
			fwrite(buffer[k].Data(), buffer[k].Length(), 1, info.files[k]);
			//ROS_DEBUG("writing in file for camera %d done", k);
		}
		else
			ROS_ERROR("Error in writing file for camera %d", k);
	}

	*/



}
