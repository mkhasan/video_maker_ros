/*
 * Util.cpp
 *
 *  Created on: Oct 18, 2019
 *      Author: usrc
 */


#include "video_maker/utils.h"
#include "stdio.h"

int64_t video_maker::SeekFile64(FILE *fid, int64_t offset, int whence) {
  if (fid == NULL) return -1;

  clearerr(fid);
  fflush(fid);
#if (defined(__WIN32__) || defined(_WIN32)) && !defined(_WIN32_WCE)
  return _lseeki64(_fileno(fid), offset, whence) == (int64_t)-1 ? -1 : 0;
#else
#if defined(_WIN32_WCE)
  return fseek(fid, (long)(offset), whence);
#else
  return fseeko(fid, (off_t)(offset), whence);
#endif
#endif
}

int64_t video_maker::TellFile64(FILE *fid) {
  if (fid == NULL) return -1;

  clearerr(fid);
  fflush(fid);
#if (defined(__WIN32__) || defined(_WIN32)) && !defined(_WIN32_WCE)
  return _telli64(_fileno(fid));
#else
#if defined(_WIN32_WCE)
  return ftell(fid);
#else
  return ftello(fid);
#endif
#endif
}
