from ...core.utils import ctypes_wrap
from .misc import default_source_message, default_placing_message, load_lib

from .uc480_lib_const import UC480_error_codes

import ctypes
import collections
import os.path
import platform

##### Constants #####

class UC480LibError(RuntimeError):
	"""Generic uc480 library error"""
	def __init__(self, func, code):
		self.func=func
		self.code=code
		self.text_code=UC480_error_codes.get(code,"UNKNOWN")
		msg="function '{}' raised error {}({})".format(func,code,self.text_code)
		RuntimeError.__init__(self,msg)
def errcheck(passing=None):
	"""
	Build an error checking function.

	Return a function which checks return codes of Andor library functions.
	`passing` is a list specifying which return codes are acceptable (by default only 20002, which is success code, is acceptable).
	"""
	passing=set(passing) if passing is not None else set()
	passing.add(0) # always allow success
	def checker(result, func, arguments):
		if result not in passing:
			raise UC480LibError(func.__name__,result)
		return result
	return checker


HCAM=ctypes.c_uint32
WORD=ctypes.c_uint16
DWORD=ctypes.c_uint32
BOOL=ctypes.c_int


class BOARDINFO(ctypes.Structure):
	_fields_=[  ("SerNo",ctypes.c_char*12),
				("ID",ctypes.c_char*20),
				("Version",ctypes.c_char*10),
				("Date",ctypes.c_char*12),
				("Select",ctypes.c_ubyte),
				("Type",ctypes.c_ubyte),
				("Reserved",ctypes.c_char*8) ]
PBOARDINFO=ctypes.POINTER(BOARDINFO)
class CBOARDINFO(ctypes_wrap.StructWrap):
	_struct=BOARDINFO
	_tup_exc={"Reserved"}


class UC480_CAMERA_INFO(ctypes.Structure):
	_fields_=[  ("dwCameraID",DWORD),
				("dwDeviceID",DWORD),
				("dwSensorID",DWORD),
				("dwInUse",DWORD),
				("SerNo",ctypes.c_char*16),
				("Model",ctypes.c_char*16),
				("dwStatus",DWORD),
				("dwReserved",DWORD*2),
				("FullModelName",ctypes.c_char*32),
				("dwReserved2",DWORD*5) ]
PUC480_CAMERA_INFO=ctypes.POINTER(UC480_CAMERA_INFO)
class CUC480_CAMERA_INFO(ctypes_wrap.StructWrap):
	_struct=UC480_CAMERA_INFO
	_tup_exc={"dwReserved","dwReserved2"}


class SENSORINFO(ctypes.Structure):
	_fields_=[  ("SensorID",WORD),
				("strSensorName",ctypes.c_char*32),
				("nColorMode",ctypes.c_byte),
				("nMaxWidth",DWORD),
				("nMaxHeight",DWORD),
				("bMasterGain",BOOL),
				("bRGain",BOOL),
				("bGGain",BOOL),
				("bBGain",BOOL),
				("bGlobShutter",BOOL),
				("wPixelSize",WORD),
				("nUpperLeftBayerPixel",ctypes.c_byte),
				("Reserved",ctypes.c_char*13) ]
PSENSORINFO=ctypes.POINTER(SENSORINFO)
class CSENSORINFO(ctypes_wrap.StructWrap):
	_struct=SENSORINFO
	_tup_exc={"Reserved"}


class UC480TIME(ctypes.Structure):
	_fields_=[  ("wYear",WORD),
				("wMonth",WORD),
				("wDay",WORD),
				("wHour",WORD),
				("wMinute",WORD),
				("wSecond",WORD),
				("wMilliseconds",WORD),
				("byReserved",ctypes.c_byte*10) ]
PUC480TIME=ctypes.POINTER(UC480TIME)
class CUC480TIME(ctypes_wrap.StructWrap):
	_struct=UC480TIME
	_tup_exc={"byReserved"}


class UC480IMAGEINFO(ctypes.Structure):
	_fields_=[  ("dwFlags",DWORD),
				("byReserved1",ctypes.c_byte*4),
				("u64TimestampDevice",ctypes.c_uint64),
				("TimestampSystem",UC480TIME),
				("dwIoStatus",DWORD),
				("wAOIIndex",WORD),
				("wAOICycle",WORD),
				("u64FrameNumber",ctypes.c_uint64),
				("dwImageBuffers",DWORD),
				("dwImageBuffersInUse",DWORD),
				("dwReserved3",DWORD),
				("dwImageHeight",DWORD),
				("dwImageWidth",DWORD),
				("dwHostProcessTime",DWORD) ]
PUC480IMAGEINFO=ctypes.POINTER(UC480IMAGEINFO)
class CUC480IMAGEINFO(ctypes_wrap.StructWrap):
	_struct=UC480IMAGEINFO
	_tup_exc={"byReserved1","dwReserved3"}
	_tup={"TimestampSystem":CUC480TIME.tup_struct}


class UC480_CAPTURE_STATUS_INFO(ctypes.Structure):
	_fields_=[  ("dwCapStatusCnt_Total",DWORD),
				("reserved",ctypes.c_byte*60),
				("adwCapStatusCnt_Detail",DWORD*256) ]
PUC480_CAPTURE_STATUS_INFO=ctypes.POINTER(UC480_CAPTURE_STATUS_INFO)
class CUC480_CAPTURE_STATUS_INFO(ctypes_wrap.StructWrap):
	_struct=UC480_CAPTURE_STATUS_INFO
	_tup_exc={"reserved"}


class IS_RECT(ctypes.Structure):
	_fields_=[  ("s32X",ctypes.c_int),
				("s32Y",ctypes.c_int),
				("s32Width",ctypes.c_int),
				("s32Height",ctypes.c_int) ]
PIS_RECT=ctypes.POINTER(IS_RECT)
class CIS_RECT(ctypes_wrap.StructWrap):
	_struct=IS_RECT

class IS_SIZE_2D(ctypes.Structure):
	_fields_=[  ("s32Width",ctypes.c_int),
				("s32Height",ctypes.c_int) ]
PIS_SIZE_2D=ctypes.POINTER(IS_SIZE_2D)
class CIS_SIZE_2D(ctypes_wrap.StructWrap):
	_struct=IS_SIZE_2D

class IS_POINT_2D(ctypes.Structure):
	_fields_=[  ("s32X",ctypes.c_int),
				("s32Y",ctypes.c_int) ]
PIS_POINT_2D=ctypes.POINTER(IS_POINT_2D)
class CIS_POINT_2D(ctypes_wrap.StructWrap):
	_struct=IS_POINT_2D




class UC480Lib(object):
	def __init__(self):
		object.__init__(self)
		self._initialized=False

	def initlib(self):
		if self._initialized:
			return
		arch=platform.architecture()[0]
		winarch="64bit" if platform.machine().endswith("64") else "32bit"
		if arch=="32bit" and winarch=="64bit":
			thorcam_path=r"C:\Program Files (x86)\Thorlabs\Scientific Imaging\ThorCam"
		else:
			thorcam_path=r"C:\Program Files\Thorlabs\Scientific Imaging\ThorCam"
		error_message="The library is supplied {};\n{}".format(default_source_message,default_placing_message)
		names=["uc480.dll" if arch[:2]=="32" else "uc480_64.dll"]
		self.lib=load_lib(names,locations=("global",thorcam_path,"local"),call_conv="stdcall",error_message=error_message,locally=True)
		lib=self.lib

		wrapper=ctypes_wrap.CTypesWrapper(restype=ctypes.c_int32,errcheck=errcheck())
		rintwrapper=ctypes_wrap.CTypesWrapper(restype=ctypes.c_int32)
		rulwrapper=ctypes_wrap.CTypesWrapper(restype=ctypes.c_ulong)


		self.is_GetNumberOfCameras=wrapper(lib.is_GetNumberOfCameras, [ctypes.c_int], [None])
		self.is_GetCameraList_lib=wrapper(lib.is_GetCameraList, [ctypes.c_void_p], ["list"])
		self.is_InitCamera=wrapper(lib.is_InitCamera, [HCAM,ctypes.c_void_p], [None,"hWnd"], addargs=["id"], rvprep=[lambda *args:HCAM(args[1])])
		self.is_ExitCamera=wrapper(lib.is_ExitCamera, [HCAM], ["hcam"])
		self.is_GetCameraInfo=wrapper(lib.is_GetCameraInfo, [HCAM, BOARDINFO], ["hcam",None], rvconv=[CBOARDINFO.tup_struct])
		self.is_GetSensorInfo=wrapper(lib.is_GetSensorInfo, [HCAM, SENSORINFO], ["hcam",None], rvconv=[CSENSORINFO.tup_struct])
		self.is_GetDLLVersion=rintwrapper(lib.is_GetDLLVersion)
		self.is_GetError=rintwrapper(lib.is_GetError, [HCAM,ctypes.c_int,ctypes.c_char_p], ["hcam",None,None], rvnames=["code","name"])
		self.is_CameraStatus=rulwrapper(lib.is_CameraStatus, [HCAM,ctypes.c_int,ctypes.c_ulong], ["hcam","ninfo","value"])
		self.is_CaptureStatus=rulwrapper(lib.is_CaptureStatus, [HCAM,ctypes.c_uint,ctypes.c_void_p,ctypes.c_uint], ["hcam","comm","param","size"])

		self.is_GetFrameTimeRange=wrapper(lib.is_GetFrameTimeRange, [HCAM,ctypes.c_double,ctypes.c_double,ctypes.c_double], ["hcam",None,None,None], rvnames=["min_time","max_time","step_time"])
		self.is_GetFramesPerSecond=wrapper(lib.is_GetFramesPerSecond, [HCAM,ctypes.c_double], ["hcam",None])
		self.is_SetFrameRate=wrapper(lib.is_SetFrameRate, [HCAM,ctypes.c_double,ctypes.c_double], ["hcam","fps",None])
		self.is_PixelClock=wrapper(lib.is_PixelClock, [HCAM,ctypes.c_uint,ctypes.c_void_p,ctypes.c_uint], ["hcam","comm","param","size"])
		self.is_Exposure=wrapper(lib.is_Exposure, [HCAM,ctypes.c_uint,ctypes.c_void_p,ctypes.c_uint], ["hcam","comm","param","size"])
		self.is_SetSubSampling=rintwrapper(lib.is_SetSubSampling, [HCAM,ctypes.c_int], ["hcam","mode"])
		self.is_SetBinning=rintwrapper(lib.is_SetBinning, [HCAM,ctypes.c_int], ["hcam","mode"])
		self.is_AOI=wrapper(lib.is_AOI, [HCAM,ctypes.c_uint,ctypes.c_void_p,ctypes.c_uint], ["hcam","comm","param","size"])
		self.is_SetColorMode=rintwrapper(lib.is_SetColorMode, [HCAM,ctypes.c_int], ["hcam","mode"])
		self.is_GetColorDepth=wrapper(lib.is_GetColorDepth, [HCAM,ctypes.c_int,ctypes.c_int], ["hcam",None,None], rvnames=["bpp","mode"])
		self.is_Blacklevel=wrapper(lib.is_Blacklevel, [HCAM,ctypes.c_uint,ctypes.c_void_p,ctypes.c_uint], ["hcam","comm","param","size"])
		self.is_GetUsedBandwidth=rintwrapper(lib.is_GetUsedBandwidth, [HCAM], ["hcam"])
		self.is_SetHardwareGain=rintwrapper(lib.is_SetHardwareGain, [HCAM,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int], ["hcam","master","red","green","blue"])
		self.is_SetGainBoost=rintwrapper(lib.is_SetGainBoost, [HCAM,ctypes.c_int], ["hcam","mode"])
		self.is_SetHWGainFactor=rintwrapper(lib.is_SetHWGainFactor, [HCAM,ctypes.c_int,ctypes.c_int], ["hcam","mode","factor"])
		self.is_SetHardwareGamma=rintwrapper(lib.is_SetHardwareGamma, [HCAM,ctypes.c_int], ["hcam","gamma"])
		self.is_SetCameraID=rintwrapper(lib.is_SetCameraID, [HCAM,ctypes.c_int], ["hcam","id"])
		self.is_HotPixel=wrapper(lib.is_HotPixel, [HCAM,ctypes.c_uint,ctypes.c_void_p,ctypes.c_uint], ["hcam","comm","param","size"])
		

		self.is_SetExternalTrigger=rintwrapper(lib.is_SetExternalTrigger, [HCAM,ctypes.c_int], ["hcam","mode"])
		self.is_SetTriggerCounter=rintwrapper(lib.is_SetTriggerCounter, [HCAM,ctypes.c_int], ["hcam","counter"])
		self.is_SetTriggerDelay=rintwrapper(lib.is_SetTriggerDelay, [HCAM,ctypes.c_int], ["hcam","delay"])
		self.is_ForceTrigger=rintwrapper(lib.is_ForceTrigger, [HCAM], ["hcam"])

		self.is_CaptureVideo=wrapper(lib.is_CaptureVideo, [HCAM,ctypes.c_int], ["hcam","wait"])
		self.is_FreezeVideo=wrapper(lib.is_FreezeVideo, [HCAM,ctypes.c_int], ["hcam","wait"])
		self.is_StopLiveVideo=wrapper(lib.is_StopLiveVideo, [HCAM,ctypes.c_int], ["hcam","wait"])
		self.is_HasVideoStarted=wrapper(lib.is_HasVideoStarted, [HCAM,ctypes.c_int], ["hcam",None])
		self.is_IsVideoFinish=wrapper(lib.is_IsVideoFinish, [HCAM,ctypes.c_int], ["hcam",None])

		self.is_AllocImageMem=wrapper(lib.is_AllocImageMem, [HCAM,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_void_p,ctypes.c_int], ["hcam","width","height","bitsperpixel",None,None], rvnames=["buff","id"])
		self.is_GetImageMemPitch=wrapper(lib.is_GetImageMemPitch, [HCAM,ctypes.c_int], ["hcam",None])
		self.is_SetAllocatedImageMem=wrapper(lib.is_SetAllocatedImageMem, [HCAM,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_void_p,ctypes.c_int], ["hcam","width","height","bitsperpixel","buff",None], rvnames=["id"])
		self.is_FreeImageMem=wrapper(lib.is_FreeImageMem, [HCAM,ctypes.c_void_p,ctypes.c_int], ["hcam","buff","id"])
		self.is_SetImageMem=wrapper(lib.is_SetImageMem, [HCAM,ctypes.c_void_p,ctypes.c_int], ["hcam","buff","id"])
		self.is_CopyImageMem=wrapper(lib.is_CopyImageMem, [HCAM,ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p], ["hcam","src","id","dst"])
		self.is_CopyImageMemLines=wrapper(lib.is_CopyImageMemLines, [HCAM,ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_void_p], ["hcam","src","id","nlines","dst"])
		self.is_GetImageInfo=rulwrapper(lib.is_GetImageInfo, [HCAM,ctypes.c_uint,UC480IMAGEINFO,ctypes.c_int], ["hcam","id",None,None],
			rvprep=[None,lambda *args:ctypes.sizeof(UC480IMAGEINFO)], rvnames=["info",None], rvref=[True,False], rvconv=[CUC480IMAGEINFO.tup_struct,None])
		self.is_InquireImageMem=wrapper(lib.is_InquireImageMem, [HCAM,ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int],
			["hcam","buff","id",None,None,None,None], rvnames=["width","height","bits","pitch"])
			
		self.is_AddToSequence=wrapper(lib.is_AddToSequence, [HCAM,ctypes.c_void_p,ctypes.c_int], ["hcam","buff","id"])
		self.is_ClearSequence=wrapper(lib.is_ClearSequence, [HCAM], ["hcam"])
		self.is_GetActSeqBuf=wrapper(lib.is_GetActSeqBuf, [HCAM,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p], ["hcam",None,None,None], rvnames=["num_curr","buff_curr","buff_last"])
		self.is_LockSeqBuf=wrapper(lib.is_LockSeqBuf, [HCAM,ctypes.c_void_p,ctypes.c_int], ["hcam","buff","id"])
		self.is_UnlockSeqBuf=wrapper(lib.is_UnlockSeqBuf, [HCAM,ctypes.c_void_p,ctypes.c_int], ["hcam","buff","id"])

		self.is_InitImageQueue=wrapper(lib.is_InitImageQueue, [HCAM,ctypes.c_int], ["hcam","mode"])
		self.is_ExitImageQueue=wrapper(lib.is_ExitImageQueue, [HCAM], ["hcam"])
		self.is_WaitForNextImage=wrapper(lib.is_WaitForNextImage, [HCAM,ctypes.c_uint32,ctypes.c_void_p,ctypes.c_int], ["hcam","timeout",None,None], rvnames=["buff","id"])

		self._initialized=True

	def is_GetCameraList(self):
		ncam=self.is_GetNumberOfCameras()
		if ncam==0:
			return []
		class UC480_CAMERA_LIST(ctypes.Structure):
			_fields_=[  ("dwCount",ctypes.c_ulong),
						("uci",UC480_CAMERA_INFO*ncam)  ]
		cam_lst=UC480_CAMERA_LIST()
		cam_lst.dwCount=ncam
		self.is_GetCameraList_lib(ctypes.byref(cam_lst))
		if cam_lst.dwCount!=ncam:
			raise RuntimeError("unexpected number of cameras returned: expected {}, got {}".format(ncam,cam_lst.dwCount))
		return [CUC480_CAMERA_INFO.tup_struct(i) for i in cam_lst.uci]


	def is_GetCaptureStatus(self, hcam):
		status=UC480_CAPTURE_STATUS_INFO()
		self.is_CaptureStatus(hcam,2,ctypes.byref(status),ctypes.sizeof(status))
		return CUC480_CAPTURE_STATUS_INFO.tup_struct(status)
	def is_ResetCaptureStatus(self, hcam):
		self.is_CaptureStatus(hcam,1,None,0)

	def is_Exposure_d8(self, hcam, comm, value=0):
		value=ctypes.c_double(value)
		self.is_Exposure(hcam,comm,ctypes.byref(value),8)
		return value.value
	def is_PixelClock_dt(self, hcam, comm, dtype, value=0):
		value=dtype(value)
		self.is_PixelClock(hcam,comm,ctypes.byref(value),ctypes.sizeof(value))
		return value
	def is_AOI_dt(self, hcam, comm, dtype, value=0):
		if not isinstance(value,dtype):
			value=dtype(value)
		self.is_AOI(hcam,comm,ctypes.byref(value),ctypes.sizeof(value))
		return value
	def is_GetAOI(self, hcam):
		r=self.is_AOI_dt(hcam,2,IS_RECT)
		return CIS_RECT.tup_struct(r)
	def is_SetAOI(self, hcam, aoi):
		r=IS_RECT(*aoi)
		r=self.is_AOI_dt(hcam,1,IS_RECT,r)
		return CIS_RECT.tup_struct(r)
	def is_GetAOISize(self, hcam, comm):
		r=self.is_AOI_dt(hcam,comm,IS_SIZE_2D)
		return CIS_SIZE_2D.tup_struct(r)
	def is_GetAOIPos(self, hcam, comm):
		r=self.is_AOI_dt(hcam,comm,IS_POINT_2D)
		return CIS_POINT_2D.tup_struct(r)


lib=UC480Lib()