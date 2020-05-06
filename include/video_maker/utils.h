/*
 * Util.h
 *
 *  Created on: Oct 18, 2019
 *      Author: usrc
 */

#ifndef SRC_UTILS_H_
#define SRC_UTILS_H_
#include <stdio.h>
#include <cstdlib>


namespace video_maker {


int64_t SeekFile64(FILE *fid, int64_t offset, int whence);
    // A platform-independent routine for seeking within (possibly) large files

int64_t TellFile64(FILE *fid);
    // A platform-independent routine for reporting the position within
    // (possibly) large files


}

#endif /* SRC_UTILS_H_ */
