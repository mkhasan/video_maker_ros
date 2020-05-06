/*
 * JPEG_buffer.cpp
 *
 *  Created on: Oct 23, 2019
 *      Author: kict
 */



#include "ros/ros.h"

#include "video_maker/JPEG_buffer.h"
#include "client_interface/robot_exception.h"

struct jpeg_compress_struct cinfo;
struct jpeg_error_mgr jerr;
using namespace Magick;

#include <setjmp.h>



video_maker::JPEGBuffer::JPEGBuffer()
	: width(0)
	, height(0)
{

}

video_maker::MagickBuffer::MagickBuffer()
	: JPEGBuffer()

{

}

void video_maker::MagickBuffer::Fill(const shm_data *rawData) {

	width = rawData->width;
	height = rawData->height;

	static int first = 1;
	if (first) {
		ROS_DEBUG("raw len is %d", rawData->len);
		first = 0;
	}


	Blob blob(rawData->data, rawData->len);
	if (rawData->len > 0) {
		Image image(blob);
		image.write(&jpegBlob, "jpg");
	}
}
int video_maker::MagickBuffer::Length() const {
	jpegBlob.length();
}
const char * video_maker::MagickBuffer::Data() const {
	return (const char *)jpegBlob.data();
}

video_maker::FrameBuffer::FrameBuffer()
	: JPEGBuffer()
	, length(0)
{

}

void video_maker::FrameBuffer::init_JPEG()
{

	/* More stuff */
		/* target file */



  /* Step 1: allocate and initialize JPEG compression object */

	cinfo.err = jpeg_std_error(&jerr);
	/* Now we can initialize the JPEG compression object. */
	jpeg_create_compress(&cinfo);



}

void video_maker::FrameBuffer::write_JPEG_file(char * src, int offset, int image_width, int image_height, int linesize, int quality)
{
	FILE * outfile;

	int row_stride;		/* physical row width in image buffer */

	JSAMPROW row_pointer[1];	/* pointer to JSAMPLE row[s] */


	memset(data, 0, MAX_DATA_SIZE);

	if ((outfile = fmemopen(data, MAX_DATA_SIZE, "wb")) == NULL) {
			ROS_ERROR("can't open %s\n", "mem file");
			return;
	}



	//printf("widht is %d and height is %d \n", image_width, image_height);
	jpeg_stdio_dest(&cinfo, outfile);

	cinfo.image_width = image_width; 	/* image width and height, in pixels */
	cinfo.image_height = image_height;
	cinfo.input_components = 3;		/* # of color components per pixel */
	cinfo.in_color_space = JCS_RGB; 	/* colorspace of input image */

	jpeg_set_defaults(&cinfo);

	jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);

	//cinfo.scale_num = 1;
	//cinfo.scale_denom = 2;
	jpeg_start_compress(&cinfo, TRUE);

	row_stride = image_width * 3;	/* JSAMPLEs per row in image_buffer */




	JSAMPLE * image_buffer = &src[offset];

	//ROS_DEBUG("%d %d %d %d", offset, image_width, image_height, row_stride);
	while (cinfo.next_scanline < cinfo.image_height) {


		row_pointer[0] = & image_buffer[cinfo.next_scanline * row_stride];
		jpeg_write_scanlines(&cinfo, row_pointer, 1);



	}

  /* Step 6: Finish compression */

	jpeg_finish_compress(&cinfo);
	/* After finish_compress, we can close the output file. */
	length = ftello(outfile);
	fclose(outfile);





	/* Step 7: release JPEG compression object */

	/* This is an important step since it will release a good deal of memory. */

}


void video_maker::FrameBuffer::finalize_JPEG()
{
	jpeg_destroy_compress(&cinfo);
}


void video_maker::FrameBuffer::Fill(const shm_data * rawData) {
	width = rawData->width;
	height = rawData->height;

	//length = 0;	// keep previous info

	char str[256];
	sprintf(str, "P6\n%d %d\n255\n", width, height);
	int offset = strlen(str);

	if (width*3*height != rawData->len-offset) {
		THROW(RobotException, "Data format error");
	}


	init_JPEG();
	//ROS_DEBUG("raw len is %d", rawData->len);




	write_JPEG_file(rawData->data, offset, width, height, width*3, 20);
	finalize_JPEG();
	//length = GetImageLength(data);
	//ROS_DEBUG("length is %d", length);



}

int video_maker::FrameBuffer::GetImageLength(const char * data)
{
	int i=0;
	int val_1, val_2;
	for (i=0; i<MAX_DATA_SIZE-1; i++)
	{
		val_1 = (int) data[i] & 0xff;
		val_2 = (int) data[i+1] & 0xff;

		if (val_1 == 0xff && val_2 == 0xd9)
			break;
	}

	if (i == MAX_DATA_SIZE -1)
		return -1;

	return i+2;
}

int video_maker::FrameBuffer::Length() const{
	return length;

}

const char * video_maker::FrameBuffer::Data() const {
	return data;
}
