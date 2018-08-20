# from io import open

import time
import os.path
from ..utils import string, files as file_utils


class LogFile(object):
    """
    Expanding file.
    
    Args:
        path (str): Path to the destination file.
        default_fmt (list): If not ``None``, it's a defult value of `fmt` for :func:`write_dataline` method. 
    """
    def __init__(self, path, default_fmt=None):
        object.__init__(self)
        self.path=path
        self.sep="\t"
        self.default_fmt=default_fmt
        
    def _get_path(self, line, header, timestamp):
        return self.path
    def _get_timestamp_line(self, timestamp):
        return "{:.3f}".format(timestamp)
    
    def _write_line(self, path, line, header=""):
        if os.path.exists(path):
            with open(path,"a") as f:
                f.write("\n"+line)
        else:
            file_utils.ensure_dir(os.path.split(path)[0])
            with open(path,"a") as f:
                if header:
                    f.write(header+"\n")
                f.write(line)
                
    def write_line(self, line, header="", add_timestamp=True):
        """
        Write a single line into the file.
        
        Create the file if it doesn't exist.
        
        Args:
            line (str): Data line to be added.
            header (str): If non-empty, add it to the beginning of the file on creation.
            add_timestamp (bool): If ``True``, add the UNIX timestamp in the beginning of the line.
        """
        timestamp=time.time()
        if add_timestamp:
            line=self._get_timestamp_line(timestamp)+self.sep+line
        path=self._get_path(line,header,timestamp)
        self._write_line(path,line,header)
    def write_dataline(self, data, columns=None, fmt=None, add_timestamp=True):
        """
        Write a single data line into the file. 
        
        Create the file if it doesn't exist.
        
        Args:
            data (list or np.ndarray): Data row to be added.
            columns (list): If not ``None``, it's a list of column names to be added as a header on creation.
            fmt (str): If not ``None``, it's a list of format strings for the line entries.
            add_timestamp (bool): If ``True``, add the UNIX timestamp in the beginning of the line.\
        """
        self.write_multi_datalines([data],columns=columns,fmt=fmt,add_timestamp=add_timestamp)
    def write_multi_datalines(self, data, columns=None, fmt=None, add_timestamp=True):
        """
        Write a multiple data lines into the file. 
        
        Create the file if it doesn't exist.
        
        Args:
            data (list of lists): Data rows to be added.
            columns (list): If not ``None``, it's a list of column names to be added as a header on creation.
            fmt (str): If not ``None``, it's a list of format strings for the line entries.
            add_timestamp (bool): If ``True``, add the UNIX timestamp in the beginning of the line.\
        """
        if len(data)==0:
            return
        datalen=len(data[0])
        if columns:
            if len(columns)!=datalen:
                raise ValueError("columns dimensions don't agree with data dimensions")
            if add_timestamp:
                columns=["Timestamp"]+columns
            header="# "+self.sep.join(columns)
        else:
            header=""
        fmt=fmt or self.default_fmt
        if fmt is None:
            fmt=[None]*datalen
        for row in data:
            row=[string.to_string(v,location="entry") if f is None else ("{:"+f+"}").format(v) for f,v in zip(fmt,row)]
            line=self.sep.join(row)
            self.write_line(line,header,add_timestamp=add_timestamp)