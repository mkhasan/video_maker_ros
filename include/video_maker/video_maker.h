/*
 * video_maker.h
 *
 *  Created on: Oct 22, 2019
 *      Author: kict
 */

#ifndef CATKIN_WS_SRC_VIDEO_MAKER_INCLUDE_VIDEO_MAKER_VIDEO_MAKER_H_
#define CATKIN_WS_SRC_VIDEO_MAKER_INCLUDE_VIDEO_MAKER_VIDEO_MAKER_H_

#include <vector>
#include <string>

#include "video_maker/AVI_file_sink.h"

//#define WIDTH 320 //240//656//320
//#define HEIGHT 180 //160//352 //180
//#define FRAME_RATE 25
//#define True true
//#define False false
//#define MAX_DATA_SIZE 100000

#include "client_interface/shm_manager.h"

#include <ace/Shared_Memory_SV.h>

namespace video_maker{


const unsigned int FRAME_RATE = 5;
const unsigned int MIN_CLIP_DURATION = 5; 	// in sec
const unsigned int MAX_CLIP_DURATION = 600; // in sec

typedef struct {

	typedef enum {NONE=0, AUTO, SNAP_AROUND} mode_t;
	bool quit;
	std::vector<std::string> cameraNames;
	std::vector<AVIFileSink*> sinks;
	sem_t * mutexList[client_interface::ShmManager::MAX_NUM_CAMERA];
	ACE_Shared_Memory_SV shm_client[client_interface::ShmManager::MAX_NUM_CAMERA];
	struct shm_data *pData[client_interface::ShmManager::MAX_NUM_CAMERA];
	mode_t mode;
	std::string video_root;
	std::string mutexPrefix;

} info_t;



void * video_maker_func(void * ptr);
void ShmInitialize(info_t & info);
void ShmFinalize(info_t & info, int n);
void ShmFinalize(info_t & info);
void CreateFiles(const std::vector<std::string> &prefix, std::vector<FILE *> & files, info_t::mode_t curMode, const std::string & path);
void CloseFiles(std::vector<FILE *> & files);
void Update(const info_t & info);
void PrintDateTime(char *str);
char * PrintDateTime();
void UpdateLocalData(info_t & info);
void Update(info_t & info);



}	// end of namespace

#endif /* CATKIN_WS_SRC_VIDEO_MAKER_INCLUDE_VIDEO_MAKER_VIDEO_MAKER_H_ */
