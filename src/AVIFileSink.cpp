/*
 * AVIFileSink.cpp
 *
 *  Created on: Oct 18, 2019
 *      Author: usrc
 */

#include "Conf.h"
#include "Utils.h"

#include "AVIFileSink.h"

#include <iostream>
#include <assert.h>


using namespace std;

#define fourChar(x,y,z,w) ( ((w)<<24)|((z)<<16)|((y)<<8)|(x) )/*little-endian*/

#define AVIIF_LIST		0x00000001
#define AVIIF_KEYFRAME		0x00000010
#define AVIIF_NO_TIME		0x00000100
#define AVIIF_COMPRESSOR	0x0FFF0000


class FrameBuffer {
public:
	FrameBuffer(unsigned bufferSize)
    : fBufferSize(bufferSize) {
    reset();
    fData = new unsigned char[bufferSize];
  }
  virtual ~FrameBuffer() { delete[] fData; }
  void reset() { fBytesInUse = 0; }
  void addBytes(unsigned numBytes) { fBytesInUse += numBytes; }

  unsigned char* dataStart() { return &fData[0]; }
  unsigned char* dataEnd() { return &fData[fBytesInUse]; }
  unsigned bytesInUse() const { return fBytesInUse; }
  unsigned bytesAvailable() const { return fBufferSize - fBytesInUse; }

  void setPresentationTime(struct timeval const& presentationTime) {
    fPresentationTime = presentationTime;
  }
  struct timeval const& presentationTime() const {return fPresentationTime;}

private:
  unsigned fBufferSize;
  struct timeval fPresentationTime;
  unsigned char* fData;
  unsigned fBytesInUse;
};



class IOState {
public:
	IOState(AVIFileSink& sink);
	virtual ~IOState();

	void setAVIstate(unsigned subsessionIndex);
	void setFinalAVIstate();
	void AddNextFrame();



public:
  FrameBuffer *fBuffer, *fPrevBuffer;
  AVIFileSink& fOurSink;


  unsigned short fLastPacketRTPSeqNum;

  struct timeval fPrevPresentationTime;
  unsigned fMaxBytesPerSecond;
  unsigned fAVISubsessionTag;
  unsigned fAVICodecHandlerType;
  unsigned fAVIScale;
  unsigned fAVIRate;
  unsigned fAVISize;
  unsigned fNumFrames;
  unsigned fSTRHFrameCountPosition;

private:
  void useFrame(FrameBuffer& buffer);
};

class AVIIndexRecord {
public:
  AVIIndexRecord(unsigned chunkId, unsigned flags, unsigned offset, unsigned size)
    : fNext(NULL), fChunkId(chunkId), fFlags(flags), fOffset(offset), fSize(size) {
  }

  AVIIndexRecord*& next() { return fNext; }
  unsigned chunkId() const { return fChunkId; }
  unsigned flags() const { return fFlags; }
  unsigned offset() const { return fOffset; }
  unsigned size() const { return fSize; }

private:
  AVIIndexRecord* fNext;
  unsigned fChunkId;
  unsigned fFlags;
  unsigned fOffset;
  unsigned fSize;
};

AVIFileSink::AVIFileSink(Provider * provider,
			 char const* outputFileName,
			 unsigned bufferSize,
			 unsigned short movieWidth, unsigned short movieHeight,
			 unsigned movieFPS)
	: fProvider(provider), fOutputFileName(outputFileName), fBufferSize(bufferSize), fMovieWidth(movieWidth)
	, fMovieHeight(movieHeight), fMovieFPS(movieFPS)
	, fIndexRecordsHead(NULL), fIndexRecordsTail(NULL), fNumIndexRecords(0), fNumBytesWritten(0)
    , fHaveCompletedOutputFile(False)
{
	fOutFid = fopen(outputFileName, "wb");
	if (fOutFid == NULL) return;


	fCurrentIOState
      = new IOState(*this);

    if(fCurrentIOState == NULL)
    	return;


  // Begin by writing an AVI header:
    addFileHeader_AVI();
}

AVIFileSink::~AVIFileSink() {
  completeOutputFile();

  // Then, stop streaming and delete each active "AVISubsessionIOState":

    delete fCurrentIOState;


  // Then, delete the index records:
	AVIIndexRecord* cur = fIndexRecordsHead;
	while (cur != NULL) {
		AVIIndexRecord* next = cur->next();
		delete cur;
		cur = next;
	}

  // Finally, close our output file:
	fclose(fOutFid);
}

void AVIFileSink::completeOutputFile() {
	if (fHaveCompletedOutputFile || fOutFid == NULL) return;

	  // Update various AVI 'size' fields to take account of the codec data that
	  // we've now written to the file:
	unsigned maxBytesPerSecond = 0;
	unsigned numVideoFrames = 0;
	unsigned numAudioFrames = 0;

	assert(fCurrentIOState != NULL);
	IOState * ioState = fCurrentIOState;

    maxBytesPerSecond += ioState->fMaxBytesPerSecond;

	setWord(ioState->fSTRHFrameCountPosition, ioState->fNumFrames);
	numVideoFrames = ioState->fNumFrames;



	  //// Global fields:
	add4ByteString("idx1");
	addWord(fNumIndexRecords*4*4); // the size of all of the index records, which come next:
	for (AVIIndexRecord* indexRecord = fIndexRecordsHead; indexRecord != NULL; indexRecord = indexRecord->next()) {
	    addWord(indexRecord->chunkId());
	    addWord(indexRecord->flags());
	    addWord(indexRecord->offset());
	    addWord(indexRecord->size());
	}

	fRIFFSizeValue += fNumBytesWritten + fNumIndexRecords*4*4 - 4;
	setWord(fRIFFSizePosition, fRIFFSizeValue);

	setWord(fAVIHMaxBytesPerSecondPosition, maxBytesPerSecond);
	setWord(fAVIHFrameCountPosition,
	  numVideoFrames > 0 ? numVideoFrames : numAudioFrames);

	fMoviSizeValue += fNumBytesWritten;
	setWord(fMoviSizePosition, fMoviSizeValue);

	// We're done:
	fHaveCompletedOutputFile = True;
}

IOState::IOState(AVIFileSink& sink)
  : fOurSink(sink), fMaxBytesPerSecond(0), fNumFrames(0)
{

  fBuffer = new FrameBuffer(fOurSink.fBufferSize);
  fPrevBuffer = NULL;

  fPrevPresentationTime.tv_sec = 0;
  fPrevPresentationTime.tv_usec = 0;
}

void IOState::AddNextFrame() {

	Provider * provider = fOurSink.fProvider;
	assert(provider != NULL);
	struct timeval presentationTime;
	int frameSize;
	static char frameSource[MAX_DATA_SIZE];

	provider->GetData(frameSource, MAX_DATA_SIZE, frameSize, presentationTime);
	assert(frameSize > 0);



	if (fPrevPresentationTime.tv_usec != 0||fPrevPresentationTime.tv_sec != 0) {
		int uSecondsDiff
		  = (presentationTime.tv_sec - fPrevPresentationTime.tv_sec)*1000000
		  + (presentationTime.tv_usec - fPrevPresentationTime.tv_usec);
		if (uSecondsDiff > 0) {
		  unsigned bytesPerSecond = (unsigned)((frameSize*1000000.0)/uSecondsDiff);
		  if (bytesPerSecond > fMaxBytesPerSecond) {
		fMaxBytesPerSecond = bytesPerSecond;
		  }
		}
	}
	fPrevPresentationTime = presentationTime;


	AVIIndexRecord* newIndexRecord
		= new AVIIndexRecord(fAVISubsessionTag, // chunk id
			 AVIIF_KEYFRAME, // flags
			 4 + fOurSink.fNumBytesWritten, // offset (note: 4 == 'movi')
			 frameSize); // size
	fOurSink.addIndexRecord(newIndexRecord);

	  // Write the data into the file:
	fOurSink.fNumBytesWritten += fOurSink.addWord(fAVISubsessionTag);

	fOurSink.fNumBytesWritten += fOurSink.addWord(frameSize);

	fwrite(frameSource, 1, frameSize, fOurSink.fOutFid);
	fOurSink.fNumBytesWritten += frameSize;
	  // Pad to an even length:
	if (frameSize%2 != 0) fOurSink.fNumBytesWritten += fOurSink.addByte(0);

	++fNumFrames;

}

IOState::~IOState() {
	if (fBuffer != NULL)
		delete fBuffer;

	if(fPrevBuffer != NULL)
		delete fPrevBuffer;
}

void IOState::setAVIstate(unsigned subsessionIndex) {

	fAVISubsessionTag
      = fourChar('0'+subsessionIndex/10,'0'+subsessionIndex%10,'d','c');

    fAVICodecHandlerType = fourChar('m','j','p','g');

    fAVIScale = 1; // ??? #####
    fAVIRate = fOurSink.fMovieFPS; // ??? #####
    fAVISize = fOurSink.fMovieWidth*fOurSink.fMovieHeight*3; // ??? #####

}



void AVIFileSink::AddNextFrame() {

	assert(fCurrentIOState != NULL);
	fCurrentIOState->AddNextFrame();
}

////////// AVI-specific implementation //////////

void AVIFileSink::addIndexRecord(AVIIndexRecord* newIndexRecord) {
  if (fIndexRecordsHead == NULL) {
    fIndexRecordsHead = newIndexRecord;
  } else {
    fIndexRecordsTail->next() = newIndexRecord;
  }
  fIndexRecordsTail = newIndexRecord;
  ++fNumIndexRecords;
}

unsigned AVIFileSink::addWord(unsigned word) {
  // Add "word" to the file in little-endian order:
  addByte(word); addByte(word>>8);
  addByte(word>>16); addByte(word>>24);

  return 4;
}

unsigned AVIFileSink::addHalfWord(unsigned short halfWord) {
  // Add "halfWord" to the file in little-endian order:
  addByte((unsigned char)halfWord); addByte((unsigned char)(halfWord>>8));

  return 2;
}

unsigned AVIFileSink::addZeroWords(unsigned numWords) {
  for (unsigned i = 0; i < numWords; ++i) {
    addWord(0);
  }

  return numWords*4;
}

unsigned AVIFileSink::add4ByteString(char const* str) {
  addByte(str[0]); addByte(str[1]); addByte(str[2]);
  addByte(str[3] == '\0' ? ' ' : str[3]); // e.g., for "AVI "

  return 4;
}

void AVIFileSink::setWord(unsigned filePosn, unsigned size) {
  do {
    if (SeekFile64(fOutFid, filePosn, SEEK_SET) < 0) break;
    addWord(size);
    if (SeekFile64(fOutFid, 0, SEEK_END) < 0) break; // go back to where we were

    return;
  } while (0);

  // One of the SeekFile64()s failed, probable because we're not a seekable file
  cout << "AVIFileSink::setWord(): SeekFile64 failed \n";
}

// Methods for writing particular file headers.  Note the following macros:

#define addFileHeader(tag,name) \
    unsigned AVIFileSink::addFileHeader_##name() { \
        add4ByteString("" #tag ""); \
        unsigned headerSizePosn = (unsigned)TellFile64(fOutFid); addWord(0); \
        add4ByteString("" #name ""); \
        unsigned ignoredSize = 8;/*don't include size of tag or size fields*/ \
        unsigned size = 12

#define addFileHeader1(name) \
    unsigned AVIFileSink::addFileHeader_##name() { \
        add4ByteString("" #name ""); \
        unsigned headerSizePosn = (unsigned)TellFile64(fOutFid); addWord(0); \
        unsigned ignoredSize = 8;/*don't include size of name or size fields*/ \
        unsigned size = 8

#define addFileHeaderEnd \
  setWord(headerSizePosn, size-ignoredSize); \
  return size; \
}

addFileHeader(RIFF,AVI);
    size += addFileHeader_hdrl();
    size += addFileHeader_movi();
    fRIFFSizePosition = headerSizePosn;
    fRIFFSizeValue = size-ignoredSize;
addFileHeaderEnd;

addFileHeader(LIST,hdrl);
    size += addFileHeader_avih();

    // Then, add a "strl" header for each subsession (stream):
    // (Make the video subsession (if any) come before the audio subsession.)
    unsigned subsessionCount = 0;


	fCurrentIOState->setAVIstate(subsessionCount++);
	size += addFileHeader_strl();


    // Then add another JUNK entry
    ++fJunkNumber;
    size += addFileHeader_JUNK();
addFileHeaderEnd;

#define AVIF_HASINDEX           0x00000010 // Index at end of file?
#define AVIF_MUSTUSEINDEX       0x00000020
#define AVIF_ISINTERLEAVED      0x00000100
#define AVIF_TRUSTCKTYPE        0x00000800 // Use CKType to find key frames?
#define AVIF_WASCAPTUREFILE     0x00010000
#define AVIF_COPYRIGHTED        0x00020000

addFileHeader1(avih);
    unsigned usecPerFrame = fMovieFPS == 0 ? 0 : 1000000/fMovieFPS;
    size += addWord(usecPerFrame); // dwMicroSecPerFrame
    fAVIHMaxBytesPerSecondPosition = (unsigned)TellFile64(fOutFid);
    size += addWord(0); // dwMaxBytesPerSec (fill in later)
    size += addWord(0); // dwPaddingGranularity
    size += addWord(AVIF_TRUSTCKTYPE|AVIF_HASINDEX|AVIF_ISINTERLEAVED); // dwFlags
    fAVIHFrameCountPosition = (unsigned)TellFile64(fOutFid);
    size += addWord(0); // dwTotalFrames (fill in later)
    size += addWord(0); // dwInitialFrame
    size += addWord(1); // dwStreams
    size += addWord(fBufferSize); // dwSuggestedBufferSize
    size += addWord(fMovieWidth); // dwWidth
    size += addWord(fMovieHeight); // dwHeight
    size += addZeroWords(4); // dwReserved
addFileHeaderEnd;

addFileHeader(LIST,strl);
    size += addFileHeader_strh();
    size += addFileHeader_strf();
    fJunkNumber = 0;
    size += addFileHeader_JUNK();
addFileHeaderEnd;

addFileHeader1(strh);
    size += add4ByteString("vids"); // fccType
    size += addWord(fCurrentIOState->fAVICodecHandlerType); // fccHandler
    size += addWord(0); // dwFlags
    size += addWord(0); // wPriority + wLanguage
    size += addWord(0); // dwInitialFrames
    size += addWord(fCurrentIOState->fAVIScale); // dwScale
    size += addWord(fCurrentIOState->fAVIRate); // dwRate
    size += addWord(0); // dwStart
    fCurrentIOState->fSTRHFrameCountPosition = (unsigned)TellFile64(fOutFid);
    size += addWord(0); // dwLength (fill in later)
    size += addWord(fBufferSize); // dwSuggestedBufferSize
    size += addWord((unsigned)-1); // dwQuality
    size += addWord(fCurrentIOState->fAVISize); // dwSampleSize
    size += addWord(0); // rcFrame (start)

	size += addHalfWord(fMovieWidth);
	size += addHalfWord(fMovieHeight);
addFileHeaderEnd;

addFileHeader1(strf);

      // Add a BITMAPINFO header:
	unsigned extraDataSize = 0;
	size += addWord(10*4 + extraDataSize); // size
	size += addWord(fMovieWidth);
	size += addWord(fMovieHeight);
	size += addHalfWord(1); // planes
	size += addHalfWord(24); // bits-per-sample #####
	size += addWord(fCurrentIOState->fAVICodecHandlerType); // compr. type
	size += addWord(fCurrentIOState->fAVISize);
	size += addZeroWords(4); // ??? #####
      // Later, add extra data here (if any) #####


    addFileHeaderEnd;

#define AVI_MASTER_INDEX_SIZE   256

addFileHeader1(JUNK);
    if (fJunkNumber == 0) {
      size += addHalfWord(4); // wLongsPerEntry
      size += addHalfWord(0); // bIndexSubType + bIndexType
      size += addWord(0); // nEntriesInUse #####
      size += addWord(fCurrentIOState->fAVISubsessionTag); // dwChunkId
      size += addZeroWords(2); // dwReserved
      size += addZeroWords(AVI_MASTER_INDEX_SIZE*4);
    } else {
      size += add4ByteString("odml");
      size += add4ByteString("dmlh");
      unsigned wtfCount = 248;
      size += addWord(wtfCount); // ??? #####
      size += addZeroWords(wtfCount/4);
    }
addFileHeaderEnd;

addFileHeader(LIST,movi);
    fMoviSizePosition = headerSizePosn;
    fMoviSizeValue = size-ignoredSize;
addFileHeaderEnd;



