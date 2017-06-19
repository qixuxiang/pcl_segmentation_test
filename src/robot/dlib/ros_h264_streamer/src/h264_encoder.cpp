#include <ros_h264_streamer/h264_encoder.h>
#include <sensor_msgs/image_encodings.h>

extern "C"
{
  #include "x264.h"
  #include "libswscale/swscale.h"
}

namespace ros_h264_streamer
{

struct H264EncoderImpl
{
public:
  H264EncoderImpl(int width, int height, int quality_level, int fps_num, int fps_den, const std::string & encoding, bool streaming)
  : m_width(width), m_height(height), m_fps_num(fps_num), m_fps_den(fps_den), encoding(encoding)
  {
    m_stride = width*3;

    /* Parametrize x264 for real-time */
    x264_param_default_preset(&m_param, "veryfast", "animation/zerolatency");
    m_param.i_threads = 1;
    m_param.i_width = width;
    m_param.i_height = height;
    m_param.i_fps_num = m_fps_num;
    m_param.i_fps_den = m_fps_den;
    // Intra refres:
    m_param.i_keyint_max = m_fps_num/(4*m_fps_den);
    m_param.i_frame_reference = 1;
    m_param.b_intra_refresh = 1;
    //Rate control:
    m_param.rc.i_rc_method = X264_RC_CRF;
    m_param.rc.f_rf_constant = quality_level;
    m_param.rc.f_rf_constant_max = 100;

    if(streaming)
    {
      //For streaming:
      m_param.b_repeat_headers = 1;
      m_param.b_annexb = 1;
    }
    else
    {
      // Specific for video encoding
      m_param.b_vfr_input = 1;
      m_param.i_timebase_num = m_fps_den;
      m_param.i_timebase_den = m_fps_num;
      m_param.vui.i_sar_width  = 1;
      m_param.vui.i_sar_height = 1;
      m_param.i_frame_total = 0;

      m_param.b_intra_refresh = 0;
      m_param.b_repeat_headers = 0;
      m_param.b_annexb = 0;
      m_param.i_csp = X264_CSP_I420;
    }

    Init(&m_param);
  }

  ~H264EncoderImpl()
  {
    sws_freeContext(m_convert_ctx);
    x264_encoder_close(m_encoder);
  }

  H264EncoderResult encode(const sensor_msgs::ImageConstPtr & img, uint64_t pts)
  {
    x264_nal_t* nals;
    int i_nals;
    H264EncoderResult res;

    /* Convert from image encoding to YUV420P */
    uint8_t *buf_in[4]={(uint8_t*)(&(img->data[0])),NULL,NULL,NULL};
    sws_scale(m_convert_ctx, (const uint8_t* const*)buf_in, &m_stride, 0, m_height, m_pic_in.img.plane, m_pic_in.img.i_stride);
    m_pic_in.i_pts = pts;

    /* Encode */
    while( (res.frame_size = x264_encoder_encode(m_encoder, &nals, &i_nals, &m_pic_in, &m_pic_out)) == 0 ) { m_pic_in.i_pts++; }
    res.frame_data = nals[0].p_payload;

    return res;
  }

  x264_param_t * GetParameters()
  {
    return &m_param;
  }

  x264_t * GetEncoder()
  {
    return m_encoder;
  }

  void * GetPicIn()
  {
    return &m_pic_in;
  }

  void * GetPicOut()
  {
    return &m_pic_out;
  }
private:
  void Init(x264_param_t * param)
  {
    m_encoder = x264_encoder_open(param);
    m_width = param->i_width;
    m_height = param->i_height;
    m_stride = m_width*3;
    x264_picture_alloc(&m_pic_in, X264_CSP_I420, m_width, m_height);
    if(encoding == sensor_msgs::image_encodings::RGB8)
    {
      m_convert_ctx = sws_getContext(m_width, m_height, PIX_FMT_RGB24, m_width, m_height, PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    }
    else if(encoding == sensor_msgs::image_encodings::BGR8)
    {
      m_convert_ctx = sws_getContext(m_width, m_height, PIX_FMT_BGR24, m_width, m_height, PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    }
    else if(encoding == sensor_msgs::image_encodings::RGBA8)
    {
      m_stride = m_width*4;
      m_convert_ctx = sws_getContext(m_width, m_height, PIX_FMT_RGBA, m_width, m_height, PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    }
    else if(encoding == sensor_msgs::image_encodings::BGRA8)
    {
      m_stride = m_width*4;
      m_convert_ctx = sws_getContext(m_width, m_height, PIX_FMT_BGRA, m_width, m_height, PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    }
    else
    {
      std::cerr << "[ros_h264_streamer] Trying to work with unsupported encoding!" << std::endl;
    }
  }

  int m_width;
  int m_height;
  int m_fps_num;
  int m_fps_den;
  std::string encoding;
  int m_stride;

  struct SwsContext * m_convert_ctx;

  x264_param_t m_param;
  x264_t * m_encoder;
  x264_picture_t m_pic_in, m_pic_out;
};

H264Encoder::H264Encoder(int width, int height, int quality_level, int fps_num, int fps_den, const std::string & encoding, bool streaming)
: impl(new H264EncoderImpl(width, height, quality_level, fps_num, fps_den, encoding, streaming))
{
}

H264EncoderResult H264Encoder::encode(const sensor_msgs::ImageConstPtr & img, uint64_t pts)
{
  return impl->encode(img, pts);
}

x264_param_t * H264Encoder::GetParameters()
{
  return impl->GetParameters();
}

x264_t * H264Encoder::GetEncoder()
{
  return impl->GetEncoder();
}

void * H264Encoder::GetPicIn()
{
  return impl->GetPicIn();
}

void * H264Encoder::GetPicOut()
{
  return impl->GetPicOut();
}

} // namespace ros_h264_streamer
