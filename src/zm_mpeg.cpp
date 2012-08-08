/*
 * ZoneMinder MPEG class implementation, $Date$, $Revision$
 * Copyright (C) 2001-2008 Philip Coombes
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/ 

#include <stdlib.h>
#include <string.h>

#include "zm.h"
#include "zm_rgb.h"
#include "zm_mpeg.h"

#if HAVE_LIBAVCODEC

bool VideoStream::initialised = false;

VideoStream::MimeData VideoStream::mime_data[] = {
	{ "asf", "video/x-ms-asf" },
	{ "swf", "application/x-shockwave-flash" },
	{ "flv", "video/x-flv" },
	{ "mp4", "video/mp4" },
	{ "move", "video/quicktime" }
};

void VideoStream::Initialise()
{
//	avcodec_init();
	avcodec_register_all();
	av_register_all();
	initialised = true;
}

void VideoStream::SetupFormat( const char *p_filename, const char *p_format )
{
	filename = p_filename;
	format = p_format;

	/* auto detect the output format from the name. default is mpeg. */
	of = av_guess_format( format, NULL, NULL);
	if ( !of )
	{
		Warning( "Could not deduce output format from file extension: using mpeg" );
		of = av_guess_format("mpeg", NULL, NULL);
	}
	if ( !of )
	{
		Fatal( "Could not find suitable output format" );
	}
	
	/* allocate the output media context */
	ofc = (AVFormatContext *)av_mallocz(sizeof(AVFormatContext));
	if ( !ofc )
	{
		Panic( "Memory error" );
	}
	if (! strcmp( format, "asf") )
		of->video_codec =  CODEC_ID_WMV2;
		
	ofc->oformat = of;
	snprintf( ofc->filename, sizeof(ofc->filename), "%s", filename );
}

void VideoStream::SetupCodec( int colours, int subpixelorder, int width, int height, int bitrate, double frame_rate )
{
	/* ffmpeg format matching */
	switch(colours) {
	  case ZM_COLOUR_RGB24:
	  {
	    if(subpixelorder == ZM_SUBPIX_ORDER_BGR) {
	      /* BGR subpixel order */
	      pf = PIX_FMT_BGR24;
	    } else {
	      /* Assume RGB subpixel order */
	      pf = PIX_FMT_RGB24;
	    }
	    break;
	  }
	  case ZM_COLOUR_RGB32:
	  {
	    if(subpixelorder == ZM_SUBPIX_ORDER_ARGB) {
	      /* ARGB subpixel order */
	      pf = PIX_FMT_ARGB;
	    } else if(subpixelorder == ZM_SUBPIX_ORDER_ABGR) {
	      /* ABGR subpixel order */
	      pf = PIX_FMT_ABGR;
	    } else if(subpixelorder == ZM_SUBPIX_ORDER_BGRA) {
	      /* BGRA subpixel order */
	      pf = PIX_FMT_BGRA;
	    } else {
	      /* Assume RGBA subpixel order */
	      pf = PIX_FMT_RGBA;
	    }
	    break;
	  }
	  case ZM_COLOUR_GRAY8:
	    pf = PIX_FMT_GRAY8;
	    break;
	  default:
	   Debug(9, "Unexpected colours: %d",colours);
	    Panic("Unexpected colours: %d",colours);
	    break;
	}

	/* add the video streams using the default format codecs
	   and initialize the codecs */
	ost = NULL;
	if (of->video_codec != CODEC_ID_NONE)
	{
		ost = avformat_new_stream(ofc, NULL);
		ost->id = 0;
		if (!ost)
		{
		        Debug(9,"Could not alloc stream" );
			Panic( "Could not alloc stream" );
		}
		
//#if ZM_FFMPEG_SVN
		AVCodecContext *c = ost->codec;
//#else
//		AVCodecContext *c = &ost->codec;
//#endif

		c->codec_id = of->video_codec;
//#if LIBAVUTIL_VERSION_INT >= AV_VERSION_INT(51,2,1)
		c->codec_type = AVMEDIA_TYPE_VIDEO;
//#else
//		c->codec_type = CODEC_TYPE_VIDEO;
//#endif

		/* put sample parameters */
		c->bit_rate = bitrate;
		c->pix_fmt = PIX_FMT_YUV420P;
		/* resolution must be a multiple of two */
		c->width = width;
		c->height = height;
//#if ZM_FFMPEG_SVN
		/* time base: this is the fundamental unit of time (in seconds) in terms
		   of which frame timestamps are represented. for fixed-fps content,
		   timebase should be 1/framerate and timestamp increments should be
		   identically 1. */
		//c->time_base.den = (int)(frame_rate*100);
//		c->time_base.num = 1;
//		c->time_base.den = 2500;
                c->time_base= (AVRational){1,frame_rate*100};
//                c->max_b_frames=1;
                c->gop_size = 12;
		Debug(9,"Time_base Density: %d",c->time_base.den);
//                fr = 1000 * av_q2d(c->time_base);
//		c->time_base.num = 1;
//#else
//                c->time_base.num = 1;
//                c->time_base.den=25;
		/* frames per second */
//		c->frame_rate = frame_rate;
//		c->frame_rate_base = 1;
//#endif

//		  if(c->time_base.num>1000 && pCodecCtx->time_base.den==1)
		       //c->time_base.den=1000;
       
//		c->gop_size = frame_rate/2; /* emit one intra frame every half second or so */
		//c->gop_size = 1;
                Debug(9,"GOP Size: %d",c->gop_size );
//		if ( c->gop_size < 3 )
//			c->gop_size = 3;
		// some formats want stream headers to be seperate
		if(!strcmp(ofc->oformat->name, "mp4") || !strcmp(ofc->oformat->name, "mov") || !strcmp(ofc->oformat->name, "3gp") || !strcmp(ofc->oformat->name, "asf"))
			c->flags |= CODEC_FLAG_GLOBAL_HEADER;
	}
}

//void VideoStream::SetParameters()
//{
//	/* set the output parameters (must be done even if no
//	   parameters). */
//	if ( av_set_parameters(ofc, NULL) < 0 )
//	{
//		Panic( "Invalid output format parameters" );
//	}
//	//dump_format(ofc, 0, filename, 1);
//}

const char *VideoStream::MimeType() const
{
	for ( int i = 0; i < sizeof(mime_data)/sizeof(*mime_data); i++ )
	{
		if ( strcmp( format, mime_data[i].format ) == 0 )
		{
			return( mime_data[i].mime_type );
		}
	}
	const char *mime_type = of->mime_type;
	if ( !mime_type )
	{
		mime_type = "video/mpeg";
		Warning( "Unable to determine mime type for '%s' format, using '%s' as default", format, mime_type );
	}

	return( mime_type );
}

void VideoStream::OpenStream()
{
	/* now that all the parameters are set, we can open the 
	   video codecs and allocate the necessary encode buffers */
//	   Debug(9,"Starting OpenStream...");
	if ( ost )
	{
//	Debug(9,"ost is valid...");
//#if ZM_FFMPEG_SVN
		AVCodecContext *c = ost->codec;
//#else
		//AVCodecContext *c = &ost->codec;
//#endif

		/* find the video encoder */
		AVCodec *codec = avcodec_find_encoder(c->codec_id);
		if ( !codec )
		{
			Panic( "codec not found" );
		}

		/* open the codec */
		if ( avcodec_open2(c, codec, NULL) < 0 )
		{
			Panic( "Could not open codec" );
		}

		/* allocate the encoded raw picture */
		opicture = avcodec_alloc_frame();
		if ( !opicture )
		{
		        Debug(9, "Could not allocate opicture");
			Panic( "Could not allocate opicture" );
		}
		int size = avpicture_get_size( c->pix_fmt, c->width, c->height);
		uint8_t *opicture_buf = (uint8_t *)av_malloc(size);
		if ( !opicture_buf )
		{
			av_free(opicture);
			Debug(9, "Could not allocate opicture");
			Panic( "Could not allocate opicture" );
		}
		avpicture_fill( (AVPicture *)opicture, opicture_buf, c->pix_fmt, c->width, c->height );

		/* if the output format is not identical to the input format, then a temporary
		   picture is needed too. It is then converted to the required
		   output format */
		tmp_opicture = NULL;
		if ( c->pix_fmt != pf )
		{
			tmp_opicture = avcodec_alloc_frame();
			if ( !tmp_opicture )
			{
			        Debug(9, "Could not allocate temporary opicutre");
				Panic( "Could not allocate temporary opicture" );
			}
			int size = avpicture_get_size( pf, c->width, c->height);
			uint8_t *tmp_opicture_buf = (uint8_t *)av_malloc(size);
			if (!tmp_opicture_buf)
			{
				av_free( tmp_opicture );
				Debug(9,"Could not allocate temporary opicture2");
				Panic( "Could not allocate temporary opicture" );
			}
			avpicture_fill( (AVPicture *)tmp_opicture, tmp_opicture_buf, pf, c->width, c->height );
		}
	}

	/* open the output file, if needed */
	if ( !(of->flags & AVFMT_NOFILE) )
	{
#if LIBAVUTIL_VERSION_INT >= AV_VERSION_INT(51,2,1)
		 if ( avio_open(&ofc->pb, filename, AVIO_FLAG_WRITE) < 0 )
#else
		if ( url_fopen(&ofc->pb, filename, URL_WRONLY) < 0 )
#endif
		{
			Fatal( "Could not open '%s'", filename );
		}
	}

	video_outbuf = NULL;
	if ( !(ofc->oformat->flags & AVFMT_RAWPICTURE) )
	{
		/* allocate output buffer */
		/* XXX: API change will be done */
		video_outbuf_size = 3786400;
		video_outbuf = (uint8_t *)malloc(video_outbuf_size);
	}

	/* write the stream header, if any */
	if (avformat_write_header(ofc, NULL) < 0)
	{
	        Debug(9, "Invalid output format parameters or unable to write header");
		Panic( "Invalid output format parameters or unable to write header" );
	}
	
//	Debug(9,"Exiting OpenSteeam");
}

VideoStream::VideoStream( const char *filename, const char *format, int bitrate, double frame_rate, int colours, int subpixelorder, int width, int height )
{
	if ( !initialised )
	{
		Initialise();
	}

	SetupFormat( filename, format );
	SetupCodec( colours, subpixelorder, width, height, bitrate, frame_rate );
//	SetParameters();
}

VideoStream::~VideoStream()
{
	/* close each codec */
	if (ost)
	{
//#if ZM_FFMPEG_SVN
		avcodec_close(ost->codec);
//#else
		//avcodec_close(&ost->codec);
//#endif
		av_free(opicture->data[0]);
		av_free(opicture);
		if (tmp_opicture)
		{
			av_free(tmp_opicture->data[0]);
			av_free(tmp_opicture);
		}
		av_free(video_outbuf);
	}

	/* write the trailer, if any */
	av_write_trailer(ofc);
	
	/* free the streams */
	for( int i = 0; i < ofc->nb_streams; i++)
	{
		av_freep(&ofc->streams[i]);
	}

	if (!(of->flags & AVFMT_NOFILE))
	{
		/* close the output file */

		avio_close(ofc->pb);
	}

	/* free the stream */
	av_free(ofc);
}

double VideoStream::EncodeFrame( const uint8_t *buffer, int buffer_size, bool add_timestamp, unsigned int timestamp )
{
//Debug(9,"Starting Encode Frame");
#ifdef HAVE_LIBSWSCALE
    static struct SwsContext *img_convert_ctx = 0;
#endif // HAVE_LIBSWSCALE
	double pts = 0.0;

	if (ost)
	{
#if ZM_FFMPEG_048
		pts = (double)ost->pts.val * ofc->pts_num / ofc->pts_den;
#else
		pts = (double)ost->pts.val * ost->time_base.num / ost->time_base.den;
#endif
	}

//#if ZM_FFMPEG_SVN
	AVCodecContext *c = ost->codec;
//#else
	//AVCodecContext *c = &ost->codec;
//#endif
	if ( c->pix_fmt != pf )
	{
		memcpy( tmp_opicture->data[0], buffer, buffer_size );
#ifdef HAVE_LIBSWSCALE
        if ( !img_convert_ctx )
        {
            img_convert_ctx = sws_getCachedContext( NULL, c->width, c->height, pf, c->width, c->height, c->pix_fmt, SWS_BICUBIC, NULL, NULL, NULL );
            if ( !img_convert_ctx )
            {
                Debug(9,"Unable to initialise image scaling context");
                Panic( "Unable to initialise image scaling context" );
                }
        }
        sws_scale( img_convert_ctx, tmp_opicture->data, tmp_opicture->linesize, 0, c->height, opicture->data, opicture->linesize );
#else // HAVE_LIBSWSCALE
		Fatal("swscale is required for MPEG mode");
#endif // HAVE_LIBSWSCALE
	}
	else
	{
		memcpy( opicture->data[0], buffer, buffer_size );
	}
	AVFrame *opicture_ptr = opicture;

	int ret = 0;
	if ( ofc->oformat->flags & AVFMT_RAWPICTURE )
	{
//        Info("RAWPICTURE");
#if ZM_FFMPEG_048
		ret = av_write_frame( ofc, ost->index, (uint8_t *)opicture_ptr, sizeof(AVPicture) );
#else
		AVPacket pkt;
		av_init_packet( &pkt );

#if LIBAVUTIL_VERSION_INT >= AV_VERSION_INT(51,2,1)
		pkt.flags |= AV_PKT_FLAG_KEY;
#else	
		pkt.flags |= PKT_FLAG_KEY;
#endif
		pkt.stream_index = ost->index;
		pkt.data = (uint8_t *)opicture_ptr;
		pkt.size = sizeof(AVPicture);
//		Debug(9,"Writing Frame...1");
		ret = av_write_frame(ofc, &pkt);
#endif
	}
	else
	{
	        if ( add_timestamp )
	        	ost->pts.val = timestamp;

AVPacket packet;
int got_packet;
av_init_packet(&packet);
packet.data = NULL;
packet.size = 0;
got_packet = 0;
//        Info("avcodec_encode_video2");
	        ret = avcodec_encode_video2(c, &packet, opicture_ptr, &got_packet);
                if (ret < 0) {
                	//Fatal("Video Encoding Failed: %s", strerror( errno ) );
                	Debug(9,"ERROR: Encoding FAILED!");
                	Panic( "Unable to encode video!" );
		}
                if (got_packet) 
                {
//                        packet.pts= av_rescale_q( c->coded_frame->pts, c->time_base, ost->time_base );
//                        packet.pts = c->coded_frame->pts;

                        if (packet.pts != AV_NOPTS_VALUE)
                                packet.pts = av_rescale_q(opicture_ptr->best_effort_timestamp, c->time_base, ost->time_base);
                        if (packet.dts != AV_NOPTS_VALUE)
                                packet.dts = av_rescale_q(opicture_ptr->best_effort_timestamp, c->time_base, ost->time_base);

                        if(c->coded_frame->key_frame)
                       {
                              packet.flags |= AV_PKT_FLAG_KEY;
                        }


//                        Debug(9,"Writing Frame...for PTS: %f", pts);
	        	ret = av_write_frame(ofc,&packet);
//	        	ret = av_interleaved_write_frame(ofc,&packet);
        	        av_destruct_packet(&packet);
        	        avcodec_flush_buffers(c);
                }
		else
		{
//		Debug (9,"Didnt get packet...");
		}
  
  
  
                                                                    	
//		int out_size = avcodec_encode_video(c, video_outbuf, video_outbuf_size, opicture_ptr);
		
//		if ( out_size > 0 )
//		{
//#if ZM_FFMPEG_048
//			ret = av_write_frame(ofc, ost->index, video_outbuf, out_size);
//#else
//			AVPacket pkt;
//			av_init_packet(&pkt);

//#if ZM_FFMPEG_049
//			pkt.pts = c->coded_frame->pts;
//#else
//			pkt.pts= av_rescale_q( c->coded_frame->pts, c->time_base, ost->time_base );
//#endif
//			if(c->coded_frame->key_frame)
//#if LIBAVUTIL_VERSION_INT >= AV_VERSION_INT(51,2,1)
//				pkt.flags |= AV_PKT_FLAG_KEY;
//#else
//				pkt.flags |= PKT_FLAG_KEY;
//#endif
//			pkt.stream_index = ost->index;
//			pkt.data = video_outbuf;
//			pkt.size = out_size;
                        
//                        Debug(9,"Writing Frame...for PTS: %f", pts);
//			ret = av_write_frame( ofc, &pkt );
//#endif
//                }


	}
	if ( ret != 0 )
	{	
	        Debug(9,"ERROR: while writing a frame");
		Fatal( "Error %d while writing video frame: %s", ret, strerror( errno ) );
	}
//	Debug(9,"Done with Encode Frame");
	return( pts );
}

#endif // HAVE_LIBAVCODEC
