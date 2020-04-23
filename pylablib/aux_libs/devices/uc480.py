from .uc480_lib import lib, UC480LibError, errcheck

from ...core.devio.interface import IDevice
from ...core.utils import py3, general
from ...core.dataproc import image as image_utils

_depends_local=[".uc480_lib","...core.devio.interface"]

import numpy as np
import collections
import contextlib
import ctypes
import time

class UC480Error(RuntimeError):
    "Generic uc480 camera error."
class UC480TimeoutError(UC480Error):
    "Timeout while waiting."

def get_cameras_number():
    """Get number of connected uc480 cameras"""
    lib.initlib()
    return lib.is_GetNumberOfCameras()

def get_cameras_list():
    """Get list of connected uc480 cameras"""
    lib.initlib()
    return lib.is_GetCameraList()

class UC480Camera(IDevice):
    """
    Thorlabs uc480 camera.

    Args:
        idx(int): camera ID; use 0 to get the first available camera
        roi_binning_mode: determines whether binning in ROI refers to binning or subsampling;
            can be ``"bin"``, ``"subsample"``, or ``"auto"`` (since most cameras only support one, it will pick the one which has non-trivial value, or ``"bin"`` if both are available).
    """
    def __init__(self, id=0, roi_binning_mode="auto"):
        IDevice.__init__(self)
        lib.initlib()
        self.cam_id=id
        self.hcam=None
        self._buffers=None
        self._next_read_buffer=0
        self._default_nframes=100
        self._acq_params=(False,None)
        self._last_wait_frame=0
        self.image_indexing="rct"
        self.open()
        self._all_color_modes=self._check_all_color_modes()
        self._set_auto_mono_color_mode()
        if roi_binning_mode=="auto":
            if self.get_supported_binning_modes()==([1],[1]):
                roi_binning_mode="subsample"
            else:
                roi_binning_mode="bin"
        self._roi_binning_mode=roi_binning_mode

        self._add_full_info_node("model_data",self.get_model_data)
        self._add_settings_node("subsampling",self.get_subsampling,self.set_subsampling)
        self._add_full_info_node("subsampling_modes",self.get_supported_subsampling_modes)
        self._add_settings_node("binning",self.get_binning,self.set_binning)
        self._add_full_info_node("binning_modes",self.get_supported_binning_modes)
        self._add_settings_node("roi",self.get_roi,self.set_roi)
        self._add_settings_node("pixel_rate",self.get_pixel_rate,self.set_pixel_rate)
        self._add_settings_node("frame_time",self.get_frame_time,self.set_frame_time)
        self._add_settings_node("exposure",self.get_exposure,self.set_exposure)
        self._add_full_info_node("max_gains",self.get_max_gains)
        self._add_settings_node("gains",self.get_gains,self.set_gains)
        self._add_settings_node("gain_boost",self.get_gain_boost,self.set_gain_boost)
        self._add_status_node("timings",self.get_timings)
        self._add_status_node("buffer_size",self.get_buffer_size)
        self._add_status_node("buffer_status",self.get_buffer_status)
        self._add_status_node("data_dimensions",self.get_data_dimensions)
        self._add_full_info_node("pixel_rates_range",self.get_pixel_rates_range)
        self._add_full_info_node("all_color_modes",self.get_all_color_modes)
        self._add_settings_node("color_mode",self.get_color_mode,self.set_color_mode)
        self._add_full_info_node("detector_size",self.get_detector_size)
        self._add_status_node("roi_limits",self.get_roi_limits)
        self._add_status_node("acq_status",self.get_acquisition_status)
        self._add_status_node("acq_in_progress",self.acquisition_in_progress)

    def open(self):
        """Open connection to the camera"""
        if self.hcam is None:
            self.hcam=lib.is_InitCamera(None,self.cam_id)
    def close(self):
        """Close connection to the camera"""
        if self.hcam is not None:
            lib.is_ExitCamera(self.hcam)
            self.hcam=None
        self.hcam=None
    def is_opened(self):
        """Check if the device is connected"""
        return self.hcam is not None

    ModelData=collections.namedtuple("ModelData",["model","manufacturer","serial_number","usb_version","date","dll_version","camera_type"])
    def get_model_data(self):
        """
        Get camera model data.

        Return tuple ``(model, manufacturer, serial_number, usb_version, date, dll_version, camera_type)``.
        """
        sen_info=self._get_sensor_info()
        cam_info=lib.is_GetCameraInfo(self.hcam)
        dll_ver=lib.is_GetDLLVersion()
        dll_ver="{}.{}.{}".format((dll_ver>>24),(dll_ver>>16)&0xFF,dll_ver&0xFFFF)
        return self.ModelData(py3.as_str(sen_info.strSensorName),py3.as_str(cam_info.ID),py3.as_str(cam_info.SerNo),py3.as_str(cam_info.Version),
            py3.as_str(cam_info.Date),dll_ver,cam_info.Type)
    def _get_sensor_info(self):
        return lib.is_GetSensorInfo(self.hcam)

    ### Buffer controls ###
    def _allocate_buffers(self, n):
        self._deallocate_buffers()
        frame_size=self._get_data_dimensions_rc()[::-1]
        bpp=self._get_pixel_mode_settings()[0]
        self._buffers=[]
        for _ in range(n):
            self._buffers.append(lib.is_AllocImageMem(1,frame_size[0],frame_size[1],bpp))
            lib.is_AddToSequence(self.hcam,*self._buffers[-1])
        self._next_read_buffer=0
        return n
    def _deallocate_buffers(self):
        if self._buffers is not None:
            lib.is_ClearSequence(self.hcam)
            for b in self._buffers:
                lib.is_FreeImageMem(self.hcam,*b)
            self._buffers=None

    def _read_buffer(self, n):
        buff=self._buffers[n%len(self._buffers)]
        frame_info=lib.is_GetImageInfo(self.hcam,buff[1])
        bpp,nchan=self._get_pixel_mode_settings()
        shape=(frame_info.dwImageHeight,frame_info.dwImageWidth)+((nchan,) if nchan>1 else ())
        frame=np.empty(shape=shape,dtype=self._np_dtypes[bpp//nchan])
        lib.is_CopyImageMem(self.hcam,buff[0],buff[1],frame.ctypes.data)
        return frame,frame_info
    

    def get_buffer_size(self):
        """Get number of frames in the ring buffer"""
        return len(self._buffers) if self._buffers is not None else 0
    TBufferStatus=collections.namedtuple("TBufferStatus",["unread","size"])
    def get_buffer_status(self):
        unread=self._get_unread_images()
        size=self.get_buffer_size()
        return self.TBufferStatus(unread,size)
    TAcqusitionStatus=collections.namedtuple("TAcqusitionStatus",["acquired","last_read","transfer_missed"])
    def get_acquisition_status(self):
        acquired=lib.is_CameraStatus(self.hcam,2,0x8000)
        last_read=self._next_read_buffer
        cstat=lib.is_GetCaptureStatus(self.hcam).adwCapStatusCnt_Detail
        transfer_missed=sum([cstat[i] for i in [0xa2,0xa3,0xb2,0xc7]])
        return self.TAcqusitionStatus(acquired,last_read,transfer_missed)
    def _get_unread_images(self):
        acq_status=self.get_acquisition_status()
        return acq_status.acquired-acq_status.last_read


    ### Generic controls ###
    AcqTimes=collections.namedtuple("AcqTimes",["exposure","frame_time"])
    def get_timings(self):
        """
        Get acquisition timing.

        Return tuple ``(exposure, frame_time)``.
        """
        exp=lib.is_Exposure_d8(self.hcam,7)*1E-3
        frame_rate=lib.is_SetFrameRate(self.hcam,0x8000)
        return self.AcqTimes(exp,1./frame_rate)
    def set_exposure(self, exposure):
        """Set camera exposure"""
        exposure=lib.is_Exposure_d8(self.hcam,12,exposure*1E3)
        return exposure*1E-3
    def get_exposure(self):
        """Get current exposure"""
        return lib.is_Exposure_d8(self.hcam,7)*1E-3
    def set_frame_time(self, frame_time):
        """
        Set frame time (frame acquisition period).

        If the time can't be achieved and ``adjust_exposure==True``, try to adjust the exposure to get the desired frame time;
        otherwise, keep the exposure the same.
        """
        ftr=lib.is_GetFrameTimeRange(self.hcam)
        frame_time=min(max(frame_time,ftr[0]),ftr[1])
        lib.is_SetFrameRate(self.hcam,1./frame_time)
        return self.get_frame_time()
    def get_frame_time(self):
        """Get current frame time (frame acquisition period)"""
        return self.get_timings().frame_time
    def get_pixel_rate(self):
        """Get camera pixel rate (in Hz)"""
        return lib.is_PixelClock_dt(self.hcam,5,ctypes.c_uint).value*1E6
    def get_available_pixel_rates(self):
        """Get all available pixel rates (in Hz)"""
        nrates=lib.is_PixelClock_dt(self.hcam,1,ctypes.c_uint).value
        rates=lib.is_PixelClock_dt(self.hcam,2,ctypes.c_uint*nrates)[:]
        return sorted([r*1E6 for r in rates])
    def get_pixel_rates_range(self):
        """
        Get range of allowed pixel rates (in Hz).

        Return tuple ``(min, max, step)`` if minimal and maximal value, and a step.
        """
        rng=lib.is_PixelClock_dt(self.hcam,3,ctypes.c_uint*3)[:]
        return tuple([v*1E6 for v in rng])
    def set_pixel_rate(self, rate=None):
        """
        Set camera pixel rate (in Hz)

        The rate is always rounded to the closest available.
        If `rate` is ``None``, set the maximal possible rate.
        """
        rates=self.get_available_pixel_rates()
        if rate is None:
            rate=rates[-1]
        else:
            rate=sorted(rates,key=lambda r: abs(r-rate))[0]
        lib.is_PixelClock_dt(self.hcam,6,ctypes.c_uint,int(np.round(rate/1E6)))
        return self.get_pixel_rate()

    _color_modes={"raw8":11, "raw10":33, "raw12":27, "raw16":29, "mono8":6, "mono10":34, "mono12":26,"mono16":28,
                "bgr5p":3, "bgr565p":2, "rgb8p":129, "bgr8p":1, "rgba8p":128, "bgra8p":0, "rgby8p":152, "bgry8p":24,
                "rgb10p":153, "bgr10p":25, "rgb10up":163, "bgr10up":35, "rgb12up":158, "bgr12up":30, "rgba12up":159, "bgra12up":31,
                "jpeg":32, "uyuvp":12, "uyvy_monop":13, "uyuv_bayerp":14, "cbycryp":23}
    _color_modes_inv=general.invert_dict(_color_modes)
    def _check_all_color_modes(self):
        names=[]
        m0=lib.is_SetColorMode(self.hcam,0x8000)
        for n,m in self._color_modes.items():
            try:
                lib.is_SetColorMode(self.hcam,m)
                nm=lib.is_SetColorMode(self.hcam,0x8000)
                if m==nm:
                    names.append(n)
            except UC480LibError as err:
                if err.code!=174: # invalid color mode
                    raise
        lib.is_SetColorMode(self.hcam,m0)
        return names
    def get_all_color_modes(self):
        """Get a list of all available color modes"""
        return self._all_color_modes
    def get_color_mode(self):
        """
        Get current color mode.

        For possible modes, see :meth:`get_all_color_modes`.
        """
        mode=lib.is_SetColorMode(self.hcam,0x8000)
        return self._color_modes_inv.get(mode,mode)
    def set_color_mode(self, mode):
        """
        Set current color mode.

        For possible modes, see :meth:`get_all_color_modes`.
        """
        mode=self._color_modes.get(mode,mode)
        res=lib.is_SetColorMode(self.hcam,mode)
        errcheck()(res,lib.is_SetColorMode,None)
        return self.get_color_mode()
    def _set_auto_mono_color_mode(self):
        """Set color mode to the most appropriate mono setting, if the sensor is mono"""
        si=self._get_sensor_info()
        if si.nColorMode==1: # monochrome
            for mode in ["mono16","mono8"]:
                try:
                    self.set_color_mode(mode)
                    return
                except UC480LibError as err:
                    if err.code!=174: # invalid color mode
                        raise
    _mode_bpps=     {11:8, 33:16, 27:16, 29:16, 6:8, 34:16, 26:16, 28:16, 3:16, 2:16, 1:24, 0:32, 24:32, 25:32, 35:48, 30:48, 31:64, 12:16, 13:16, 14:16, 23:16}
    _mode_channels= {11:1, 33:1 , 27:1 , 29:1 , 6:1, 34:1 , 26:1 , 28:1 , 3:1 , 2:1 , 1:3 , 0:4 , 24:4 , 25:1 , 35:3 , 30:3 , 31:4 , 12:2 , 13:2 , 14:2 , 23:2 }
    def _get_pixel_mode_settings(self, mode=None):
        """.
        Get pixel mode settings (bits per pixel and channels per pixel)
        
        Packed modes are assumed to be one-channel (i.e., no unpacking is done).
        """
        if mode is None:
            mode=self.get_color_mode()
            mode=self._color_modes.get(mode,mode)
        mode&=0x7F
        return self._mode_bpps[mode],self._mode_channels[mode]
    _np_dtypes={8:"u1",16:"<u2",32:"<u4"}

    def get_gains(self):
        """
        Get current gains.

        Return tuple ``(master, red, green, blue)`` of corresponding gain factors.
        """
        return tuple([lib.is_SetHWGainFactor(self.hcam,0x8000+i,0)/100 for i in range(4)])
    def get_max_gains(self):
        """
        Get maximal gains.

        Return tuple ``(master, red, green, blue)`` of corresponding maximal gain factors.
        """
        return tuple([lib.is_SetHWGainFactor(self.hcam,0x800c+i,100)/100 for i in range(4)])
    def _set_channel_gain(self, i, ivalue):
        max_gain=lib.is_SetHWGainFactor(self.hcam,0x800c+i,100)
        min_gain=100
        ivalue=max(min(ivalue,max_gain),min_gain)
        lib.is_SetHWGainFactor(self.hcam,0x8004+i,ivalue)
    def set_gains(self, master=None, red=None, green=None, blue=None):
        """
        Set current gains.

        If supplied value is ``None``, keep it unchanged.
        """
        for i,g in enumerate([master,red,green,blue]):
            if g is not None:
                self._set_channel_gain(i,int(g*100))
        return self.get_gains()
    def get_gain_boost(self):
        """Check if gain boost is enabled"""
        return bool(lib.is_SetGainBoost(self.hcam,2) and lib.is_SetGainBoost(self.hcam,0x8008))
    def set_gain_boost(self, enabled):
        """Enabel or disable gain boost"""
        if lib.is_SetGainBoost(self.hcam,2):
            lib.is_SetGainBoost(self.hcam,1 if enabled else 0)
        return self.get_gain_boost()


    ### Acquisition process controls ###
    def start_acquisition(self, buffn=None):
        """
        Start camera acquisition.

        `buffn` specifies number of frames in the ring buffer (automatically capped at 32, which is the SDK limit)
        """
        self.stop_acquisition()
        self._allocate_buffers(n=buffn or self._default_nframes)
        lib.is_ResetCaptureStatus(self.hcam)
        lib.is_CaptureVideo(self.hcam,0)
        self._acq_params=(True,buffn)
    def stop_acquisition(self):
        """
        Stop acquisition.

        Clears buffers as well, so any readout after acquisition stop is impossible.
        """
        lib.is_StopLiveVideo(self.hcam,0)
        self._deallocate_buffers()
        self._acq_params=(False,None)
    def acquisition_in_progress(self):
        """Check if the acquisition is in progress"""
        return self._acq_params[0]
    def wait_for_frame(self, since="lastread", timeout=20., period=1E-3):
        """
        Wait for a new camera frame.

        `since` specifies what constitutes a new frame.
        Can be ``"lastread"`` (wait for a new frame after the last read frame), ``"lastwait"`` (wait for a new frame after last :meth:`wait_for_frame` call),
        or ``"now"`` (wait for a new frame acquired after this function call).
        If `timeout` is exceeded, raise :exc:`.PCOSC2TimeoutError`.
        `period` specifies camera polling period.
        """
        if not self.acquisition_in_progress():
            return
        acq_status=self.get_acquisition_status()
        last_acq_frame=acq_status.acquired-1
        last_read_frame=acq_status.last_read-1
        if since=="lastread" and last_acq_frame>last_read_frame:
            self._last_wait_frame=last_acq_frame
            return
        if since=="lastwait" and last_acq_frame>self._last_wait_frame:
            self._last_wait_frame=last_acq_frame
            return
        new_valid=False
        ctd=general.Countdown(timeout)
        while not ctd.passed():
            acq_status=self.get_acquisition_status()
            new_valid=acq_status.acquired>last_acq_frame+1
            if new_valid:
                break
            time.sleep(period)
        if not new_valid:
            raise UC480TimeoutError()
        self._last_wait_frame=acq_status.acquired-1
    @contextlib.contextmanager
    def pausing_acquisition(self):
        """
        Context manager which temporarily pauses acquisition during execution of ``with`` block.

        Useful for applying certain settings which can't be changed during the acquisition (any settings except for exposure).
        """
        acq=self.acquisition_in_progress()
        try:
            self.stop_acquisition()
            yield
        finally:
            if acq:
                self.start_acquisition()

    ### Image settings and transfer controls ###
    _subsampling_modes= {   0x0001:("v",2),0x0004:("v",4),0x0010:("v",3),0x0040:("v",5),0x0100:("v",6),0x0400:("v",8),0x1000:("v",16),
                            0x0002:("h",2),0x0008:("h",4),0x0020:("h",3),0x0080:("h",5),0x0200:("h",6),0x0800:("h",8),0x2000:("h",16) }
    _subsampling_modes_inv=general.invert_dict(_subsampling_modes)
    def _truncate_subsampling(self, hsub, vsub, all_modes):
        hsub=max(hsub,1)
        vsub=max(vsub,1)
        hmodes,vmodes=all_modes
        hsub=max([m for m in hmodes if m<=hsub])
        vsub=max([m for m in vmodes if m<=vsub])
        return hsub,vsub
    def get_supported_subsampling_modes(self):
        """
        Get all supported subsampling modes.

        Return tuple ``(horizontal, vertical)`` of lists with all possible supported subsampling factors.
        """
        all_modes=lib.is_SetSubSampling(self.hcam,0x8001)
        supp={"v":{1},"h":{1}}
        for mask,(d,s) in self._subsampling_modes.items():
            if all_modes&mask:
                supp[d].add(s)
        return sorted(supp["h"]),sorted(supp["v"])
    def get_subsampling(self):
        """Get current subsampling"""
        return lib.is_SetSubSampling(self.hcam,0x8004),lib.is_SetSubSampling(self.hcam,0x8008)
    def set_subsampling(self, hsub=1, vsub=1):
        """
        Set subsampling.
        
        If values are not supported, get the closest value below the requested.
        """
        hsub,vsub=self._truncate_subsampling(hsub,vsub,self.get_supported_subsampling_modes())
        mask=self._subsampling_modes_inv.get(("h",hsub),0)|self._subsampling_modes_inv.get(("v",vsub),0)
        res=lib.is_SetSubSampling(self.hcam,mask)
        errcheck()(res,lib.is_SetSubSampling,None)
        return self.get_subsampling()


    def get_supported_binning_modes(self):
        """
        Get all supported binning modes.

        Return tuple ``(horizontal, vertical)`` of lists with all possible supported binning factors.
        """
        all_modes=lib.is_SetBinning(self.hcam,0x8001)
        supp={"v":{1},"h":{1}}
        for mask,(d,s) in self._subsampling_modes.items():
            if all_modes&mask:
                supp[d].add(s)
        return sorted(supp["v"]),sorted(supp["h"])
    def get_binning(self):
        """Get current binning"""
        return lib.is_SetBinning(self.hcam,0x8004),lib.is_SetBinning(self.hcam,0x8008)
    def set_binning(self, hbin=1, vbin=1):
        """
        Set binning.
        
        If values are not supported, get the closest value below the requested.
        """
        hbin,vbin=self._truncate_subsampling(hbin,vbin,self.get_supported_binning_modes())
        mask=self._subsampling_modes_inv.get(("h",hbin),0)|self._subsampling_modes_inv.get(("v",vbin),0)
        res=lib.is_SetBinning(self.hcam,mask)
        errcheck()(res,lib.is_SetBinning,None)
        return self.get_binning()

    def get_detector_size(self):
        """Get camera detector size (in pixels) as a tuple ``(width, height)``"""
        sensor=self._get_sensor_info()
        return sensor.nMaxWidth,sensor.nMaxHeight
    def _check_aoi(self, aoi):
        lib.is_SetAOI(self.hcam,aoi)
        return tuple(lib.is_GetAOI(self.hcam))==aoi
    def _get_roi_binning(self):
        return self.get_subsampling() if self._roi_binning_mode=="subsample" else self.get_binning()
    def _set_roi_binning(self, hbin, vbin):
        if self._roi_binning_mode=="subsample":
            self.set_subsampling(hbin,vbin)
            self.set_binning()
        else:
            self.set_subsampling()
            self.set_binning(hbin,vbin)
    def _truncate_roi_binning(self, hbin, vbin):
        all_modes=self.get_supported_subsampling_modes() if self._roi_binning_mode=="subsample" else self.get_supported_binning_modes()
        return self._truncate_subsampling(hbin,vbin,all_modes)
    def _get_roi_limits(self):
        smin=lib.is_GetAOISize(self.hcam,8)
        sstep=lib.is_GetAOISize(self.hcam,18)
        pstep=lib.is_GetAOIPos(self.hcam,17)
        return (smin[0],sstep[0],pstep[0]),(smin[1],sstep[1],pstep[1])
    def _adj_roi_axis(self, start, end, minsize, detsize, wstep, pstep, bin):
        if end is None:
            end=detsize
        start//=bin
        end//=bin
        detsize//=bin
        end=min(end,detsize)
        start=min(start,detsize)
        start-=start%pstep
        end-=end%pstep
        end-=(end-start)%wstep
        if end-start<minsize:
            end=start+minsize
        if end>detsize:
            end=detsize
            start=detsize-minsize
        return start*bin,end*bin
    def _trunc_roi(self, hstart=0, hend=None, vstart=0, vend=None, hbin=1, vbin=1):
        wdet,hdet=self.get_detector_size()
        hbin,vbin=self._truncate_roi_binning(hbin,vbin)
        hlims,vlims=self._get_roi_limits()
        hstart,hend=self._adj_roi_axis(hstart,hend,hlims[0],wdet,hlims[1],hlims[2],hbin)
        vstart,vend=self._adj_roi_axis(vstart,vend,vlims[0],hdet,vlims[1],vlims[2],vbin)
        return hstart,hend,vstart,vend,hbin,vbin
    def get_roi(self):
        """
        Get current ROI.

        Return tuple ``(hstart, hend, vstart, vend, hbin, vbin)``.
        """
        rect=lib.is_GetAOI(self.hcam)
        hbin,vbin=self._get_roi_binning()
        return (rect.s32X*hbin,(rect.s32X+rect.s32Width)*hbin,rect.s32Y*vbin,(rect.s32Y+rect.s32Height)*vbin,hbin,vbin)
    def set_roi(self, hstart=0, hend=None, vstart=0, vend=None, hbin=1, vbin=1):
        """
        Setup camera ROI.

        `hstart` and `hend` specify horizontal image extent, `vstart` and `vend` specify vertical image extent
        (start are inclusive, stop are exclusive, starting from 0), `hbin` and `vbin` specify binning.
        By default, all non-supplied parameters take extreme values.
        """
        roi=hstart,hend,vstart,vend,hbin,vbin
        hstart,hend,vstart,vend,hbin,vbin=self._trunc_roi(*roi)
        self._set_roi_binning(hbin,vbin)
        lib.is_SetAOI(self.hcam,(hstart//hbin,vstart//vbin,(hend-hstart)//hbin,(vend-vstart)//vbin))
        return self.get_roi()
    def get_roi_limits(self):
        """
        Get the minimal and maximal ROI parameters.

        Return tuple ``(min_roi, max_roi)``, where each element is in turn 6-tuple describing the ROI.
        """
        wdet,hdet=self.get_detector_size()
        hlims,vlims=self._get_roi_limits()
        hbin,vbin=self._truncate_roi_binning(wdet,hdet)
        min_roi=(0,0,hlims[0],vlims[0],1,1)
        max_roi=(wdet-hlims[0],wdet-vlims[0],wdet,hdet,hbin,vbin)
        return (min_roi,max_roi)

    def _get_data_dimensions_rc(self):
        roi=self.get_roi()
        return (roi[3]-roi[2])//roi[5],(roi[1]-roi[0])//roi[4]
    def get_data_dimensions(self):
        """Get readout data dimensions"""
        return image_utils.convert_shape_indexing(self._get_data_dimensions_rc(),"rc",self.image_indexing)
    
    def get_new_images_range(self):
        """
        Get the range of the new images.
        
        Return tuple ``(first, last)`` with images range (inclusive).
        If no images are available, return ``None``.
        If some images were in the buffer were overwritten, exclude them from the range.
        """
        if not self._buffers:
            return None
        acq_status=self.get_acquisition_status()
        if acq_status.last_read==acq_status.acquired:
            return None
        last_read=max(acq_status.last_read,acq_status.acquired-len(self._buffers))
        return last_read,acq_status.acquired-1
    def read_multiple_images(self, rng=None, return_info=False):
        """
        Read multiple images specified by `rng` (by default, all un-read images).

        If ``return_info==True``, return tuple ``(images, info)``, where ``images`` is a list of frames,
        and ``info`` is a list of frame info tuples with frame internal index, timestamp, etc.
        If ``return_info==False``, just return a list of frames.
        Each frame is a 2D (for single-channel) or 3D (for multi-channel) numpy array, wich channel being the last (i.e., third) axis.
        Note that some color modes (BGR5, BGR565, and RGB/BGR10 packed) aren't automatically unpacked, so they result in a single-channel image.
        """
        acq_status=self.get_acquisition_status()
        new_images_rng=self.get_new_images_range()
        if rng is None:
            rng=new_images_rng
        if rng is None:
            return [] if return_info else [],[]
        rng=list(rng)
        rng[0]=max(rng[0],acq_status.acquired-len(self._buffers))
        rng[1]=min(rng[1],acq_status.acquired-1)
        if rng is None:
            return [] if return_info else [],[]
        self._next_read_buffer=max(self._next_read_buffer,rng[1]+1)
        frames=[self._read_buffer(i) for i in range(rng[0],rng[1]+1)]
        imgs,infos=list(zip(*frames))
        imgs=[image_utils.convert_image_indexing(im,"rct",self.image_indexing) for im in imgs]
        if return_info:
            return imgs,infos
        else:
            return imgs

    ### Combined functions ###
    def snap(self, n=1, buffn=None):
        """Snap n images"""
        self.start_acquisition(buffn=buffn)
        frames=[]
        while len(frames)<n:
            self.wait_for_frame()
            frames+=self.read_multiple_images()
        self.stop_acquisition()
        return frames[:n]