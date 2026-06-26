#include <nvbufsurface.h>

typedef void *CUarray;

typedef enum CUeglFrameType_enum {
  CU_EGL_FRAME_TYPE_ARRAY = 0,
  CU_EGL_FRAME_TYPE_PITCH = 1,
} CUeglFrameType;

typedef enum CUeglColorFormat_enum {
  CU_EGL_COLOR_FORMAT_UNKNOWN = 0,
} CUeglColorFormat;

typedef enum CUarray_format_enum {
  CU_AD_FORMAT_UNSIGNED_INT8 = 1,
} CUarray_format;

typedef struct CUeglFrame_st {
  union {
    CUarray pArray[3];
    void *pPitch[3];
  } frame;
  unsigned int width;
  unsigned int height;
  unsigned int depth;
  unsigned int pitch;
  unsigned int planeCount;
  unsigned int numChannels;
  CUeglFrameType frameType;
  CUeglColorFormat eglColorFormat;
  CUarray_format cuFormat;
} CUeglFrame;
