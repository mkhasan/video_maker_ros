/*
 * AVIFileSink.h
 *
 *  Created on: Oct 18, 2019
 *      Author: usrc
 */

#ifndef SRC_AVIFILESINK_H_
#define SRC_AVIFILESINK_H_

#include "Conf.h"

#include "JPEGVideoSource.h"

#include <string>








class AVIFileSink {


public:

	AVIFileSink( Provider * provider,
				 char const* outputFileName,
				 unsigned bufferSize = MAX_DATA_SIZE,
				 unsigned short movieWidth = WIDTH, unsigned short movieHeight = HEIGHT,
				 unsigned movieFPS = FRAME_RATE);

	void AddNextFrame();
	void Close();

	virtual ~AVIFileSink();

	void addIndexRecord(class AVIIndexRecord* newIndexRecord);
	void completeOutputFile();

private:
	friend class IOState;



	Provider * fProvider;
	std::string fOutputFileName;

	FILE* fOutFid;
	class AVIIndexRecord *fIndexRecordsHead, *fIndexRecordsTail;
	unsigned fNumIndexRecords;
	unsigned fNumBytesWritten;
	Boolean fHaveCompletedOutputFile;

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
	unsigned short fMovieWidth, fMovieHeight;
	unsigned fMovieFPS;
	unsigned fRIFFSizePosition, fRIFFSizeValue;
	unsigned fAVIHMaxBytesPerSecondPosition;
	unsigned fAVIHFrameCountPosition;
	unsigned fMoviSizePosition, fMoviSizeValue;
	class IOState* fCurrentIOState;
	unsigned fJunkNumber;

};
#endif /* SRC_AVIFILESINK_H_ */
