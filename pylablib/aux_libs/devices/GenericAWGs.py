from ...core.devio import SCPI, units  #@UnresolvedImport

import contextlib

_depends_local=["...core.devio.SCPI"]


class GenericAWG(SCPI.SCPIDevice):
    """
    Generic arbitrary wave generator, based on Agilent 33500.

    With slight modifications works for many other AWGs using largely the same syntax.
    """
    _channels_number=1
    _exclude_commands=set()
    _single_channel_commands=set()
    _all_channel_commands=set()
    _set_angle_unit=True
    _default_operation_cooldown=0.01
    def __init__(self, addr):
        SCPI.SCPIDevice.__init__(self,addr,backend="auto")
        self._set_channels_number()
        functions={"SIN":"sine","SQU":"square","RAMP":"ramp","PULS":"pulse","NOIS":"noise","PRBS":"prbs","DC":"DC","USER":"user","ARB":"arb"}
        for ch in range(1,self._channels_number+1):
            self._add_scpi_parameter("output_on","",kind="bool",channel=ch,comm_kind="output",add_node=True)
            self._add_scpi_parameter("output_polarity","POLARITY",kind="enum",options={"NORM":"norm","INV":"inv"},channel=ch,comm_kind="output",add_node=True)
            self._add_scpi_parameter("output_sync","SYNC",kind="bool",channel=ch,comm_kind="output",add_node=True)
            self._add_settings_node("load",self.get_load,self.set_load,channel=ch)
            self._add_settings_node("range",self.get_range,self.set_range,multiarg=False,channel=ch)
            self._add_settings_node("frequency",self.get_frequency,self.set_frequency,channel=ch)
            self._add_settings_node("phase",self.get_phase,self.set_phase,channel=ch)
            self._add_scpi_parameter("function","FUNCTION",kind="enum",options=functions,channel=ch,add_node=True)
            self._add_scpi_parameter("duty_cycle","FUNCTION:SQUARE:DCYCLE",channel=ch,add_node=True)
            self._add_scpi_parameter("ramp_symmetry","FUNCTION:RAMP:SYMMETRY",channel=ch,add_node=True)
            self._add_scpi_parameter("pulse_width","FUNCTION:PULSE:WIDTH",channel=ch,add_node=True)
            self._add_scpi_parameter("burst_enabled","BURST:STATE",kind="bool",channel=ch,add_node=True)
            self._add_scpi_parameter("burst_mode","BURST:MODE",kind="enum",options={"TRIG":"trig","GAT":"gate"},channel=ch,add_node=True)
            self._add_settings_node("burst_ncycles",self.get_burst_ncycles,self.set_burst_ncycles,channel=ch)
            self._add_scpi_parameter("gate_polarity","BURST:GATE:POL",kind="enum",options={"NORM":"norm","INV":"inv"},channel=ch,set_delay=0.1,add_node=True)
            self._add_scpi_parameter("trigger_source","TRIG:SOURCE",kind="enum",options={"IMM":"imm","EXT":"ext","BUS":"bus"},channel=ch,add_node=True)
            self._add_scpi_parameter("trigger_slope","TRIG:SLOPE",kind="enum",options={"POS":"pos","NEG":"neg"},channel=ch,add_node=True)
            self._add_scpi_parameter("trigger_output","OUTPUT:TRIG",kind="bool",channel=ch,add_node=True)
            self._add_scpi_parameter("output_trigger_slope","OUTPUT:TRIG:SLOPE",kind="enum",options={"POS":"pos","NEG":"neg"},channel=ch,add_node=True)
        self._current_channel=1

    def _set_channels_number(self):
        pass
    def _can_add_command(self, name, channel=1):
        return not (name in self._exclude_commands or (name in self._single_channel_commands and channel>1))
    def _add_scpi_parameter(self, name, comm, kind="float", options=None, match_option="prefix", set_delay=0, channel=None, comm_kind="source", add_node=False):
        if channel is not None and name not in self._all_channel_commands:
            if not self._can_add_command(name,channel):
                return
            name=self._build_channel_node(name,channel)
            comm=self._build_channel_command(comm,channel,kind=comm_kind)
        return SCPI.SCPIDevice._add_scpi_parameter(self,name,comm,kind=kind,options=options,match_option=match_option,set_delay=set_delay,add_node=add_node)
    def _add_settings_node(self, path, getter=None, setter=None, ignore_error=(), mux=None, multiarg=True, channel=None):
        if channel is not None and path not in self._all_channel_commands:
            if not self._can_add_command(path,channel):
                return
            path=self._build_channel_node(path,channel)
            if getter is not None:
                ogetter=getter
                getter=lambda *args: ogetter(*args,channel=channel)
            if getter is not None:
                osetter=setter
                setter=lambda *args: osetter(*args,channel=channel)
        return SCPI.SCPIDevice._add_settings_node(self,path,getter=getter,setter=setter,ignore_error=ignore_error,mux=mux,multiarg=multiarg)
    def _get_channel_scpi_parameter(self, name, channel=None):
        if name in self._all_channel_commands:
            return self._get_scpi_parameter(name)
        channel=self._get_channel(channel)
        return self._get_scpi_parameter(self._build_channel_node(name,channel))
    def _set_channel_scpi_parameter(self, name, value, channel=None):
        if name in self._all_channel_commands:
            return self._set_scpi_parameter(name,value)
        channel=self._get_channel(channel)
        return self._set_scpi_parameter(self._build_channel_node(name,channel),value)
    def _ask_channel(self, msg, data_type="string", delay=0., timeout=None, read_echo=False, name=None, channel=None, comm_kind="source"):
        if name not in self._all_channel_commands:
            msg=self._build_channel_command(msg,channel,kind=comm_kind)
        return SCPI.SCPIDevice.ask(self,msg,data_type=data_type,delay=delay,timeout=timeout,read_echo=read_echo)
    def _write_channel(self, msg, arg=None, arg_type=None, unit=None, fmt=None, bool_selector=("OFF","ON"), read_echo=False, read_echo_delay=0., name=None, channel=None, comm_kind="source"):
        if name not in self._all_channel_commands:
            msg=self._build_channel_command(msg,channel,kind=comm_kind)
        return SCPI.SCPIDevice.write(self,msg,arg=arg,arg_type=arg_type,unit=unit,fmt=fmt,bool_selector=bool_selector,read_echo=read_echo,read_echo_delay=read_echo_delay)

    def _build_channel_node(self, node, channel):
        """Build channel-specific settings node path"""
        if self._channels_number==1:
            return node
        else:
            return "ch{}/{}".format(channel,node)
    def _build_channel_command(self, comm, channel, kind="source"):
        """Build channel-specific command"""
        channel=self._get_channel(channel)
        if kind=="source":
            if self._channels_number==1:
                return comm
            return "SOURCE{}:{}".format(channel,comm)
        elif kind=="output":
            if self._channels_number==1:
                pfx="OUTPUT"
            else:
                pfx="OUTPUT{}".format(channel)
            return pfx+":"+comm if comm else pfx
    def _check_ch(self, channel=None):
        if channel is not None and (channel<1 or channel>self._channels_number):
            raise ValueError("invalid channel: {}".format(channel))
    def _get_channel(self, channel=None):
        self._check_ch(channel)
        return self._current_channel if channel is None else channel

    def get_channel(self):
        return self._current_channel
    def select_channel(self, channel):
        self._check_ch(channel)
        self._current_channel=channel
    @contextlib.contextmanager
    def default_channel(self, channel):
        curr_ch=self._current_channel
        try:
            self.select_channel(channel)
            yield
        finally:
            self.select_channel(curr_ch)
    
    def get_output(self, channel=None):
        """Check if the output is enabled"""
        return self._get_channel_scpi_parameter("output_on",channel=channel)
    def set_output(self, enabled=True, channel=None):
        """Turn the output on or off"""
        return self._set_channel_scpi_parameter("output_on",enabled,channel=channel)

    
    def get_output_polarity(self, channel=None):
        """
        Get output polarity.

        Can be either ``"norm"`` or ``"inv"``.
        """
        return self._get_channel_scpi_parameter("output_polarity",channel=channel)
    def set_output_polarity(self, polarity="norm", channel=None):
        """
        Set output polarity.

        Can be either ``"norm"`` or ``"inv"``.
        """
        return self._set_channel_scpi_parameter("output_polarity",polarity,channel=channel)
    def is_sync_output_enabled(self, channel=None):
        """Check if SYNC output is enabled"""
        return self._get_channel_scpi_parameter("sync_output",channel=channel)
    def enable_sync_output(self, enabled=True, channel=None):
        """Enable or disable SYNC output"""
        return self._set_channel_scpi_parameter("sync_output",enabled,channel=channel)
        
    def get_load(self, channel=None):
        """Get the output load"""
        return self._ask_channel("LOAD?","float",name="output_load",channel=channel,comm_kind="output")
    def set_load(self, load=None, channel=None):
        """Set the output load (``None`` means High-Z)"""
        load="INF" if load is None else load
        self._write_channel("LOAD",load,name="output_load",channel=channel,comm_kind="output")
        return self.get_load(channel=channel)
        
    def get_function(self, channel=None):
        """
        Get output function.

        Can be one of the following: ``"sine"``, ``"square"``, ``"ramp"``, ``"pulse"``, ``"noise"``, ``"prbs"``, ``"DC"``, ``"user"``, ``"arb"``.
        Not all functions can be available, depending on the particular model of the generator.
        """
        return self._get_channel_scpi_parameter("function",channel=channel)
    def set_function(self, func, channel=None):
        """
        Set output function.

        Can be one of the following: ``"sine"``, ``"square"``, ``"ramp"``, ``"pulse"``, ``"noise"``, ``"prbs"``, ``"DC"``, ``"user"``, ``"arb"``.
        Not all functions can be available, depending on the particular model of the generator.
        """
        return self._set_channel_scpi_parameter("function",func,channel=channel)
    
    def get_amplitude(self, channel=None):
        """Get output amplitude"""
        self._write_channel("VOLTAGE:UNIT","VPP",name="voltage_unit",channel=channel)
        return self._ask_channel("VOLTAGE?","float",name="amplitude",channel=channel)/2.
    def set_amplitude(self, amplitude, channel=None):
        """Set output amplitude"""
        self._write_channel("VOLTAGE:UNIT","VPP",name="voltage_unit",channel=channel)
        self._write_channel("VOLTAGE",amplitude*2,"float",name="amplitude",channel=channel)
        return self.get_amplitude(channel=channel)
    def get_offset(self, channel=None):
        """Get output offset"""
        return self._ask_channel("VOLTAGE:OFFSET?","float",name="offset",channel=channel)
    def set_offset(self, offset, channel=None):
        """Set output offset"""
        self._write_channel("VOLTAGE:OFFSET",offset,"float",name="offset",channel=channel)
        return self.get_offset(channel=channel)
    def get_range(self, channel=None):
        """
        Get output voltage range.
        
        Return tuple ``(vmin, vmax)`` with the low and high voltage values (i.e., ``offset-amplitude`` and ``offset+amplitude``).
        """
        low=self._ask_channel("VOLTAGE:LOW?","float",name="voltage",channel=channel)
        high=self._ask_channel("VOLTAGE:HIGH?","float",name="voltage",channel=channel)
        return low,high
    def set_range(self, rng, channel=None):
        """
        Set output voltage range.
        
        If span is less than ``1E-4``, automatically switch to DC mode.
        """
        try:
            low,high=min(rng),max(rng)
        except TypeError:
            low,high=rng,rng
        if abs(high-low)<1E-4:
            self.set_function("DC",channel=channel)
            self.set_amplitude(10E-3,channel=channel)
            self.set_offset((high+low)/2.,channel=channel)
        else:
            curr_rng=self.get_range(channel=channel)
            if low<curr_rng[1]:
                self._write_channel("VOLTAGE:LOW",low,"float",name="voltage",channel=channel)
                self._write_channel("VOLTAGE:HIGH",high,"float",name="voltage",channel=channel)
            else:
                self._write_channel("VOLTAGE:HIGH",high,"float",name="voltage",channel=channel)
                self._write_channel("VOLTAGE:LOW",low,"float",name="voltage",channel=channel)
        return self.get_range(channel=channel)
    
    def get_frequency(self, channel=None):
        """Get output frequency"""
        value,unit=self._ask_channel("FREQUENCY?","value",name="frequency",channel=channel)
        return units.convert_frequency_units(value,unit or "Hz","Hz")
    def set_frequency(self, frequency, channel=None):
        """Set output frequency"""
        self._write_channel("FREQUENCY",frequency,"float",name="frequency",channel=channel)
        return self.get_frequency(channel=channel)
    def get_phase(self, channel=None):
        """Get output phase (in degrees)"""
        if self._set_angle_unit:
            self.write(":UNIT:ANGLE DEG")
        return self._ask_channel("PHASE?","float",name="phase",channel=channel)
    def set_phase(self, phase, channel=None):
        """Set output phase (in degrees)"""
        if self._set_angle_unit:
            self.write(":UNIT:ANGLE DEG")
        self._write_channel("PHASE",phase,"float",name="phase",channel=channel)
        return self.get_phase(channel=channel)
    def sync_phase(self):
        """Synchronize phase between two channels"""
        self.write("SOURCE1:PHASE:SYNC")
    
    def get_duty_cycle(self, channel=None):
        """
        Get output duty cycle (in percent).

        Only applies to ``"square"`` output function.
        """
        return self._get_channel_scpi_parameter("duty_cycle",channel=channel)
    def set_duty_cycle(self, dcycle, channel=None):
        """
        Set output duty cycle (in percent).

        Only applies to ``"square"`` output function.
        """
        return self._set_channel_scpi_parameter("duty_cycle",dcycle,channel=channel)
    def get_ramp_symmetry(self, channel=None):
        """
        Get output ramp symmetry (in percent).

        Only applies to ``"ramp"`` output function.
        """
        return self._get_channel_scpi_parameter("ramp_symmetry",channel=channel)
    def set_ramp_symmetry(self, rsymm, channel=None):
        """
        Set output ramp symmetry (in percent).

        Only applies to ``"ramp"`` output function.
        """
        return self._set_channel_scpi_parameter("ramp_symmetry",rsymm,channel=channel)
    def get_pulse_width(self, channel=None):
        """
        Get output pulse width (in seconds).

        Only applies to ``"pulse"`` output function.
        """
        return self._get_channel_scpi_parameter("pulse_width",channel=channel)
    def set_pulse_width(self, width, channel=None):
        """
        Set output pulse width (in seconds).

        Only applies to ``"pulse"`` output function.
        """
        return self._set_channel_scpi_parameter("pulse_width",width,channel=channel)

    def is_burst_enabled(self, channel=None):
        """Check if the burst mode is enabled"""
        return self._get_channel_scpi_parameter("burst_enabled",channel=channel)
    def enable_burst(self, enabled=True, channel=None):
        """Enable burst mode"""
        return self._set_channel_scpi_parameter("burst_enabled",enabled,channel=channel)
    def get_burst_mode(self, channel=None):
        """
        Get burst mode.

        Can be either ``"trig"`` or ``"gate"``.
        """
        return self._get_channel_scpi_parameter("burst_mode",channel=channel)
    def set_burst_mode(self, mode, channel=None):
        """
        Set burst mode.

        Can be either ``"trig"`` or ``"gate"``.
        """
        return self._set_channel_scpi_parameter("burst_mode",mode,channel=channel)
    def get_burst_ncycles(self, channel=None):
        """
        Get burst mode ncycles.

        Infinite corresponds to a large value (>1E37).
        """
        return self._ask_channel("BURST:NCYC?","int",name="burst_ncycles",channel=channel)
    def set_burst_ncycles(self, ncycles=1, channel=None):
        """
        Set burst mode ncycles.

        Infinite corresponds to ``None``
        """
        ncycles="INF" if ncycles is None or ncycles>1E37 else ncycles
        self._write_channel("BURST:NCYC",int(ncycles),"int",name="burst_ncycles",channel=channel)
        self.sleep(0.1)
        return self.get_burst_ncycles(channel=channel)
    def get_gate_polarity(self, channel=None):
        """
        Get burst gate polarity.

        Can be either ``"norm"`` or ``"inv"``.
        """
        return self._get_channel_scpi_parameter("gate_polarity",channel=channel)
    def set_gate_polarity(self, polarity="norm", channel=None):
        """
        Set burst gate polarity.

        Can be either ``"norm"`` or ``"inv"``.
        """
        return self._set_channel_scpi_parameter("gate_polarity",polarity,channel=channel)
    
    
    def get_trigger_source(self, channel=None):
        """
        Get trigger source.

        Can be either ``"imm"``, ``"ext"``, or ``"bus"``.
        """
        return self._get_channel_scpi_parameter("trigger_source",channel=channel)
    def set_trigger_source(self, src, channel=None):
        """
        Set trigger source.

        Can be either ``"imm"``, ``"ext"``, or ``"bus"``.
        """
        return self._set_channel_scpi_parameter("trigger_source",src,channel=channel)
    def get_trigger_slope(self, channel=None):
        """
        Get trigger slope.

        Can be either ``"pos"``, or ``"neg"``.
        """
        return self._get_channel_scpi_parameter("trigger_slope",channel=channel)
    def set_trigger_slope(self, slope, channel=None):
        """
        Set trigger slope.

        Can be either ``"pos"``, or ``"neg"``.
        """
        return self._set_channel_scpi_parameter("trigger_slope",slope,channel=channel)
    def is_trigger_output_enabled(self, channel=None):
        """Check if the trigger output is enabled"""
        return self._get_channel_scpi_parameter("trigger_output",channel=channel)
    def enable_trigger_output(self, enabled=True, channel=None):
        """Enable trigger output"""
        return self._set_channel_scpi_parameter("trigger_output",enabled,channel=channel)
    def get_output_trigger_slope(self, channel=None):
        """
        Get output trigger slope.

        Can be either ``"pos"``, or ``"neg"``.
        """
        return self._get_channel_scpi_parameter("output_trigger_slope",channel=channel)
    def set_output_trigger_slope(self, slope, channel=None):
        """
        Set output trigger slope.

        Can be either ``"pos"``, or ``"neg"``.
        """
        return self._set_channel_scpi_parameter("output_trigger_slope",slope,channel=channel)








class Agilent33500(GenericAWG):
    """
    Agilent 33500 AWG.

    Args:
        channels_number: number of channels; if ``"auto"``, try to deremine automatically (by certain commands causing errors)
    """
    def __init__(self, addr, channels_number="auto"):
        self._channels_number=channels_number
        GenericAWG.__init__(self,addr)
    def _set_channels_number(self):
        if self._is_command_valid("OUTPUT2?"):
            self._channels_number=2
        else:
            self._channels_number=1



class Agilent33220A(GenericAWG):
    """
    Agilent 33220A AWG.
    """
    pass



class InstekAFG2225(GenericAWG):
    """
    Instek AFG-2225 AWG.
    """
    _exclude_commands={"output_polarity","output_sync","trigger_output","output_trigger_slope"}
    _set_angle_unit=False
    _channels_number=2
    def __init__(self, addr):
        GenericAWG.__init__(self,addr)
        for ch in range(1,self._channels_number+1):
            self._add_scpi_parameter("duty_cycle","SQUARE:DCYCLE",channel=ch,add_node=True)
            self._add_scpi_parameter("ramp_symmetry","RAMP:SYMMETRY",channel=ch,add_node=True)
            self._add_scpi_parameter("pulse_width","PULSE:WIDTH",channel=ch,add_node=True)
            self._add_scpi_parameter("trigger_source","BURST:TRIG:SOURCE",kind="enum",options={"IMM":"imm","EXT":"ext","BUS":"bus"},channel=ch,add_node=True)
            self._add_scpi_parameter("trigger_slope","BURST:TRIG:SLOPE",kind="enum",options={"POS":"pos","NEG":"neg"},channel=ch,add_node=True)
    def get_amplitude(self, channel=None):
        """Get output amplitude"""
        if self._ask_channel("VOLTAGE:UNIT?",name="voltage_unit",channel=channel).upper()!="VPP":
            self._write_channel("VOLTAGE:UNIT","VPP",name="voltage_unit",channel=channel)
        return self._ask_channel("AMPLITUDE?","float",name="amplitude",channel=channel)/2.
    def set_amplitude(self, amplitude, channel=None):
        """Set output amplitude"""
        if self._ask_channel("VOLTAGE:UNIT?",name="voltage_unit",channel=channel).upper()!="VPP":
            self._write_channel("VOLTAGE:UNIT","VPP",name="voltage_unit",channel=channel)
            self._write_channel("AMPLITUDE",amplitude*2,"float",name="amplitude",channel=channel)
            self.sleep(1) # it looks like one needs to wait some time after setting the amplitude; otherwise, there's no response to the next command
        else:
            self._write_channel("AMPLITUDE",amplitude*2,"float",name="amplitude",channel=channel)
        return self.get_amplitude(channel=channel)
    def get_offset(self, channel=None):
        """Get output offset"""
        return self._ask_channel("DCOFFSET?","float",name="offset",channel=channel)
    def set_offset(self, offset, channel=None):
        """Set output offset"""
        self._write_channel("DCOFFSET",offset,"float",name="offset",channel=channel)
        return self.get_offset(channel=channel)
    def get_range(self, channel=None):
        """
        Get output voltage range.
        
        Return tuple ``(vmin, vmax)`` with the low and high voltage values (i.e., ``offset-amplitude`` and ``offset+amplitude``).
        """
        amp=self.get_amplitude(channel=channel)
        off=self.get_offset(channel=channel)
        return off-amp,off+amp
    def set_range(self, rng, channel=None):
        """
        Set output voltage range.
        
        If span is less than ``1E-4``, automatically switch to DC mode.
        """
        try:
            low,high=min(rng),max(rng)
        except TypeError:
            low,high=rng,rng
        if abs(high-low)<1E-4:
            self.set_function("DC",channel=channel)
            self.set_amplitude(10E-3,channel=channel)
            self.set_offset((high+low)/2.,channel=channel)
        else:
            amp,off=(rng[1]-rng[0])/2,(rng[1]+rng[0])/2
            curr_amp=self.get_amplitude(channel=channel)
            if curr_amp>=amp:
                self.set_amplitude(amp,channel=channel)
                self.set_offset(off,channel=channel)
            else:
                self.set_offset(off,channel=channel)
                self.set_amplitude(amp,channel=channel)
        return self.get_range(channel=channel)
        


class RigolDG1000(GenericAWG):
    """
    Rigol DG1000 AWG.
    """
    _default_operation_cooldown=0.05
    _channels_number=2
    _single_channel_commands={"output_sync",
        "burst_enabled","burst_mode","burst_ncycles","gate_polarity",
        "trigger_source","trigger_slope","trigger_output","output_trigger_slope"}
    _set_angle_unit=False
    def __init__(self, addr):
        GenericAWG.__init__(self,addr)
        for ch in range(1,self._channels_number+1):
            self._add_scpi_parameter("pulse_width","PULSE:WIDTH",channel=ch,add_node=True)
    def _instr_read(self, raw=False):
        data=GenericAWG._instr_read(self,raw=raw)
        if not raw:
            if data.startswith(b"CH1:"):
                data=data[4:].strip()
            if data.startswith(b"CH2:"):
                data=data[4:].strip()
        return data
    def _build_channel_command(self, comm, channel, kind="source"):
        """Build channel-specific command"""
        channel=self._get_channel(channel)
        sfx=""
        if comm.endswith("?"):
            sfx="?"
            comm=comm[:-1]
        if kind=="output":
            comm="OUTPUT:"+comm if comm else "OUTPUT"
        if channel==2:
            comm=comm+":CH2"
        return comm+sfx
    def sync_phase(self):
        """Synchronize phase between two channels"""
        self.write("PHASE:ALIGN")
        