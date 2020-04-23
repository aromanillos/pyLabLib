from .extlibs.thorlabs_tsi_sdk import tl_camera

from ...core.devio.interface import IDevice
from ...core.utils import py3, general
from ...core.dataproc import image as image_utils

_depends_local=["...core.devio.interface"]

import numpy as np
import collections
import contextlib
import ctypes
import time

class ThorlabsTSIError(RuntimeError):
    "Generic Thorlabs TSI camera error."
class ThorlabsTSITimeoutError(ThorlabsTSIError):
    "Timeout while waiting."

sdk=None
_open_cameras=0

def get_cameras_number():
    """Get number of connected uc480 cameras"""
    return len(get_cameras_list())

def get_cameras_list():
    """Get list of connected Thorlabs TSI cameras"""
    if sdk is not None:
        return sdk.discover_available_cameras()
    else:
        with tl_camera.TLCameraSDK() as lsdk:
            return lsdk.discover_available_cameras()

class ThorlabsTSICamera(IDevice):
    """
    Thorlabs TSI camera.

    Args:
        serial(str): camera serial number; can be either a string obtained using :func:`get_cameras_list` function,
            or ``None``, which means connecting to the first available camera (not recommended unless only one camera is connected)
    """
    def __init__(self, serial=None):
        IDevice.__init__(self)
        lst=get_cameras_list()
        if serial is None:
            serial=lst[0] if lst else ""
        elif serial not in lst:
            print(serial,lst)
            raise ThorlabsTSIError("camera with serial number {} isn't present among available cameras: {}".format(serial,lst))
        self.serial=serial
        self.cam=None
        self._buffer=None
        self._default_nframes=100
        self._last_wait_frame=0
        self.image_indexing="rct"
        self.open()

        self.cam.image_poll_timeout_ms=0
        
        self._add_full_info_node("model_data",self.get_model_data)
        self._add_settings_node("roi",self.get_roi,self.set_roi)
        self._add_settings_node("frame_time",self.get_frame_time)
        self._add_settings_node("exposure",self.get_exposure,self.set_exposure)
        self._add_status_node("timings",self.get_timings)
        self._add_status_node("buffer_size",self.get_buffer_size)
        self._add_status_node("buffer_status",self.get_buffer_status)
        self._add_status_node("data_dimensions",self.get_data_dimensions)
        self._add_full_info_node("detector_size",self.get_detector_size)
        self._add_status_node("roi_limits",self.get_roi_limits)
        self._add_status_node("acq_status",self.get_acquisition_status)
        self._add_status_node("acq_in_progress",self.acquisition_in_progress)

    def open(self):
        """Open connection to the camera"""
        global sdk, _open_cameras
        try:
            if self.cam is None:
                if sdk is None:
                    sdk=tl_camera.TLCameraSDK()
                self.cam=sdk.open_camera(self.serial)
                _open_cameras+=1
        finally:
            if _open_cameras==0 and sdk is not None:
                sdk.dispose()
                sdk=None
    def close(self):
        """Close connection to the camera"""
        global sdk, _open_cameras
        if self.cam is not None:
            self.stop_acquisition()
            self.cam.dispose()
            _open_cameras-=1
            if _open_cameras==0 and sdk is not None:
                sdk.dispose()
        self.cam=None
    def is_opened(self):
        """Check if the device is connected"""
        return self.cam is not None

    ModelData=collections.namedtuple("ModelData",["model","name","serial_number","firmware_version"])
    def get_model_data(self):
        """
        Get camera model data.

        Return tuple ``(model, name, serial_number, firmware_version)``.
        """
        return self.ModelData(self.cam.model,self.cam.name,self.cam.serial_number,self.cam.firmware_version)

    # ### Acquisition controls ###
    class RingBuffer:
        def __init__(self, size):
            self.size=size
            self.buffer=[]
            self.last_read_idx=0
            self.last_added_idx=0
            self.next_read_idx=None
            self.dropped=0
        def add(self, frame):
            frame=(frame.image_buffer.copy(),frame.frame_count)
            self.buffer.append(frame)
            self.last_added_idx=frame[1]
            if self.next_read_idx is None:
                self.next_read_idx=self.last_added_idx
            if len(self.buffer)>self.size:
                del self.buffer[0]
                self.next_read_idx=self.buffer[0][1]
                self.dropped+=1
        def pop(self):
            if self.buffer:
                frame=self.buffer[0]
                del self.buffer[0]
                self.next_read_idx=self.buffer[0][1] if self.buffer else None
                self.last_read_idx=frame[1]
                return frame
        TBufferStatus=collections.namedtuple("TBufferStatus",["size","filled","last_added_idx","last_read_idx","next_read_idx","dropped"])
        def get_status(self):
            return self.TBufferStatus(self.size,len(self.buffer),self.last_added_idx,self.last_read_idx,self.next_read_idx,self.dropped)
    
    def _check_buffer(self):
        if self._buffer:
            while True:
                frame=self.cam.get_pending_frame_or_null()
                if frame is not None:
                    self._buffer.add(frame)
                else:
                    break
    

    def get_buffer_size(self):
        """Get number of frames in the ring buffer"""
        return self._buffer.get_status().size if self._buffer is not None else 0
    TBufferStatus=collections.namedtuple("TBufferStatus",["unread","size"])
    def get_buffer_status(self):
        if self._buffer is not None:
            self._check_buffer()
            bs=self._buffer.get_status()
            return self.TBufferStatus(bs.size-bs.filled,bs.size)
        return self.TBufferStatus(0,0)
    TAcqusitionStatus=collections.namedtuple("TAcqusitionStatus",["acquired","last_read","next_read"])
    def get_acquisition_status(self):
        if self._buffer is not None:
            self._check_buffer()
            bs=self._buffer.get_status()
            acquired=bs.last_added_idx
            last_read=bs.last_read_idx-1
            next_read=bs.next_read_idx-1 if bs.next_read_idx is not None else None 
            return self.TAcqusitionStatus(acquired,last_read,next_read)
        else:
            return self.TAcqusitionStatus(0,0,None)

    AcqTimes=collections.namedtuple("AcqTimes",["exposure","frame_time"])
    def get_timings(self):
        """
        Get acquisition timing.

        Return tuple ``(exposure, frame_time)``.
        """
        return self.cam.exposure_time_us*1E-6, self.cam.frame_time_us*1E-6
    def set_exposure(self, exposure):
        """Set camera exposure"""
        self.cam.exposure_time_us=int(exposure/1E-6)
        return self.get_exposure()
    def get_exposure(self):
        """Get current exposure"""
        return self.cam.exposure_time_us*1E-6
    def get_frame_time(self):
        """Get current frame time (frame acquisition period)"""
        return self.cam.frame_time_us*1E-6

    ### Acquisition process controls ###
    def start_acquisition(self, buffn=None):
        """
        Start camera acquisition.

        `buffn` specifies number of frames in the ring buffer (automatically capped at 32, which is the SDK limit)
        """
        self.stop_acquisition()
        buffn=buffn or self._default_nframes
        self._buffer=self.RingBuffer(buffn+10)
        self.cam.frames_per_trigger_zero_for_unlimited=0
        self.cam.arm(buffn)
        self.cam.issue_software_trigger()
    def stop_acquisition(self):
        """
        Stop acquisition.

        Clears buffers as well, so any readout after acquisition stop is impossible.
        """
        self.cam.disarm()
        self._buffer=None
    def acquisition_in_progress(self):
        """Check if the acquisition is in progress"""
        return self.cam.is_armed
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
            raise ThorlabsTSITimeoutError()
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
    def get_detector_size(self):
        """Get camera detector size (in pixels) as a tuple ``(width, height)``"""
        return self.cam.sensor_width_pixels,self.cam.sensor_height_pixels
    def get_roi(self):
        """
        Get current ROI.

        Return tuple ``(hstart, hend, vstart, vend, hbin, vbin)``.
        """
        roi=self.cam.roi
        return (roi[0],roi[2]+1,roi[1],roi[3]+1,self.cam.binx,self.cam.biny)
    def set_roi(self, hstart=0, hend=None, vstart=0, vend=None, hbin=1, vbin=1):
        """
        Setup camera ROI.

        `hstart` and `hend` specify horizontal image extent, `vstart` and `vend` specify vertical image extent
        (start are inclusive, stop are exclusive, starting from 0), `hbin` and `vbin` specify binning.
        By default, all non-supplied parameters take extreme values.
        """
        self.cam.binx=hbin
        self.cam.biny=vbin
        if hend is None:
            hend=self.cam.sensor_width_pixels
        if vend is None:
            vend=self.cam.sensor_height_pixels
        self.cam.roi=(hstart,vstart,hend-1,vend-1)
        return self.get_roi()
    def get_roi_limits(self):
        """
        Get the minimal and maximal ROI parameters.

        Return tuple ``(min_roi, max_roi)``, where each element is in turn 6-tuple describing the ROI.
        """
        wdet,hdet=self.get_detector_size()
        hmin=self.cam.image_width_range_pixels.min
        vmin=self.cam.image_height_range_pixels.min
        hbin=self.cam.binx_range.max
        vbin=self.cam.biny_range.max
        min_roi=(0,0,hmin,vmin,1,1)
        max_roi=(wdet-hmin,wdet-vmin,wdet,hdet,hbin,vbin)
        return (min_roi,max_roi)

    def _get_data_dimensions_rc(self):
        return self.cam.image_height_pixels,self.cam.image_width_pixels
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
        if not self._buffer:
            return None
        acq_status=self.get_acquisition_status()
        if acq_status.next_read is None:
            return None
        return acq_status.next_read,acq_status.acquired-1
    def read_multiple_images(self, rng=None, return_info=False):
        """
        Read multiple images specified by `rng` (by default, all un-read images).

        If ``return_info==True``, return tuple ``(images, info)``, where ``images`` is a list of frames,
        and ``info`` is a list of frame info tuples extracted from the binary status line (with only one member, frame index).
        If ``return_info==False``, just return a list of frames.

        For technical reasons, frames should be read in successively, and every frame can only be read once.
        Hence, if `rng` is specified, it can lead to either skipping unread frames (if `rng` starts after the first unread frame),
        or reduced number of frames compared to request (if `rng` attempts to read non-acquired or already-read frames).
        If some frames in the middle of the range are missin, they are going to be replaced by ``None``.
        """
        new_images_rng=self.get_new_images_range()
        if rng is None:
            rng=new_images_rng
        if rng is None:
            return [] if return_info else [],[]
        rng=list(rng)
        acq_status=self.get_acquisition_status()
        rng[0]=max(rng[0],acq_status.next_read)
        rng[1]=min(rng[1],acq_status.acquired-1)
        if rng[0]>new_images_rng[0]:
            acq_status=self.get_acquisition_status()
            while acq_status.next_read<rng[0]:
                self._buffer.pop()
                acq_status=self.get_acquisition_status()
            rng[0]=acq_status.next_read
        frames=[]
        for i in range(rng[0],rng[1]+1):
            if self._buffer.get_status().next_read_idx==i+1:
                frames.append(self._buffer.pop())
            else:
                frames.append((None,None))
        imgs,infos=list(zip(*frames))
        imgs=[image_utils.convert_image_indexing(im,"rct",self.image_indexing) if im is not None else None for im in imgs]
        if return_info:
            return imgs,infos
        else:
            return imgs

    ### Combined functions ###
    def snap(self, n=1, buffn=None, return_info=False):
        """Snap n images"""
        self.start_acquisition(buffn=buffn)
        frames=[]
        infos=[]
        while len(frames)<n:
            self.wait_for_frame()
            nframes,ninfos=self.read_multiple_images(return_info=True)
            frames+=nframes
            infos+=ninfos
        self.stop_acquisition()
        return (frames[:n],infos[:n]) if return_info else frames[:n]