/*
 * JPEGBuffer.h
 *
 *  Created on: Oct 23, 2019
 *      Author: kict
 */

#ifndef CATKIN_WS_SRC_VIDEO_MAKER_INCLUDE_VIDEO_MAKER_JPEGBUFFER_H_
#define CATKIN_WS_SRC_VIDEO_MAKER_INCLUDE_VIDEO_MAKER_JPEGBUFFER_H_


#include "client_interface/shm_manager.h"


#include <Magick++.h>
#include <Magick++/Blob.h>


#ifdef __cplusplus
extern "C" {
#endif

#include <jpeglib.h>
#include <libavutil/frame.h>

#ifdef __cplusplus
}
#endif



namespace video_maker {

class JPEGBuffer {
public:
	JPEGBuffer();
	virtual void Fill(const shm_data *data) = 0;
	virtual int Length() const =0;
	virtual const char * Data() const = 0;
	virtual int GetBufferSize() const{
		return MAX_DATA_SIZE;
	}

	int Width() const {
		return width;
	}
	int Height() const {
		return height;
	}



protected:
	int width;
	int height;

	char data[MAX_DATA_SIZE];
};

class MagickBuffer: public JPEGBuffer {
public:

	MagickBuffer();
	void Fill(const shm_data *rawData);
	int Length() const;
	const char * Data() const;

private:
	Magick::Blob jpegBlob;


};

class FrameBuffer: public JPEGBuffer {

public:
	FrameBuffer();
	void Fill(const shm_data *rawData);
	int Length() const;
	const char * Data() const;

private:
	void init_JPEG();
	void write_JPEG_file(char * src, int offset, int image_width, int image_height, int linesize, int quality);
	void finalize_JPEG();
	int GetImageLength(const char * data);


	int length;
};

}

#endif /* CATKIN_WS_SRC_VIDEO_MAKER_INCLUDE_VIDEO_MAKER_JPEGBUFFER_H_ */
