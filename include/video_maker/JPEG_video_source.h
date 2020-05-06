/*
 * JPEGSource.h
 *
 *  Created on: Oct 18, 2019
 *      Author: usrc
 */

#ifndef SRC_JPEGVIDEOSOURCE_H_
#define SRC_JPEGVIDEOSOURCE_H_

#include "client_interface/shm_data.h"
#include <sys/time.h>
#include <stdio.h>


typedef unsigned int u_int32_t;
typedef int int32_t;

#include "video_maker/JPEG_buffer.h"

namespace video_maker {
class Source {

public:
	Source() {

	}
	virtual ~Source() {

	}

	virtual const char * GetData(int &dataLen, struct timeval& resultPresentationTime, const void * ptr = NULL) = 0;
	virtual void Reset() = 0;

protected:
	struct timeval fPresentationTime;

};

class JPEGVideoSource : public Source {
	const double FREQ = 90000.0;
	const double EPSILON = 0.01;
public:
	JPEGVideoSource(video_maker::JPEGBuffer & frameBuffer, unsigned movieFPS);
	virtual const char * GetData(int &dataLen, struct timeval& resultPresentationTime, const void * ptr = NULL );

	void Reset();

	unsigned GetMovieFPS() const {
		return fMovieFPS;
	}
	virtual ~JPEGVideoSource();

private:
	unsigned fMovieFPS;
	video_maker::JPEGBuffer & fFrameBuffer;
	static int GetFileLength(const char * filename);
	static int ReadFile(const char * filename, char * data, int maxLen);


private:

	u_int32_t fSeqNo;
	u_int32_t fSyncTimestamp;
	u_int32_t fTimestamp;
	struct timeval fSyncTime;



};

}

#endif /* SRC_JPEGSOURCE_H_ */
