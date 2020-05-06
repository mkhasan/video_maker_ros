/*
 * AVI_file_sink.h
 *
 *  Created on: Oct 24, 2019
 *      Author: kict
 */

#ifndef CATKIN_WS_SRC_VIDEO_MAKER_INCLUDE_VIDEO_MAKER_AVI_FILE_SINK_H_
#define CATKIN_WS_SRC_VIDEO_MAKER_INCLUDE_VIDEO_MAKER_AVI_FILE_SINK_H_



#include "JPEG_video_source.h"

#include "client_interface/shm_data.h"
#include <string>






namespace video_maker {

class AVIFileSink {


public:

	AVIFileSink( Source * source,
				 unsigned bufferSize,
				 unsigned movieWidth, unsigned movieHeight,
				 unsigned movieFPS);


	void Initialize(FILE *outFid);
	void Initialize(char const* outputFileName);

	void Finalize();

	void AddNextFrame(const shm_data & rawData);
	void Close();

	virtual ~AVIFileSink();

	void addIndexRecord(class AVIIndexRecord* newIndexRecord);


private:
	void completeOutputFile();
	bool isInitialized;
	void Initialize();

	friend class IOState;



	Source * fSource;


	FILE* fOutFid;
	class AVIIndexRecord *fIndexRecordsHead, *fIndexRecordsTail;
	unsigned fNumIndexRecords;
	unsigned fNumBytesWritten;
	bool fHaveCompletedOutputFile;

  ///// Definitions specific to the AVI file format:

   unsigned addWord(unsigned word); // outputs "word" in little-endian order
   unsigned addHalfWord(unsigned short halfWord);
   unsigned addByte(unsigned char byte) {
     putc(byte, fOutFid);
     return 1;
   }
   unsigned addZeroWords(unsigned numWords);
   unsigned add4ByteString(char const* str);
   void setWord(unsigned filePosn, unsigned size);

   // Define member functions for outputting various types of file header:
 #define _header(name) unsigned addFileHeader_##name()
   _header(AVI);
       _header(hdrl);
           _header(avih);
           _header(strl);
               _header(strh);
               _header(strf);
               _header(JUNK);
 //        _header(JUNK);
       _header(movi);
 private:

    unsigned fBufferSize;
	unsigned fMovieWidth, fMovieHeight;
	unsigned fMovieFPS;
	unsigned fRIFFSizePosition, fRIFFSizeValue;
	unsigned fAVIHMaxBytesPerSecondPosition;
	unsigned fAVIHFrameCountPosition;
	unsigned fMoviSizePosition, fMoviSizeValue;
	class IOState* fCurrentIOState;
	unsigned fJunkNumber;

};


}

#endif /* CATKIN_WS_SRC_VIDEO_MAKER_INCLUDE_VIDEO_MAKER_AVI_FILE_SINK_H_ */
