"""
A wrapper for built-in TCP/IP routines.
"""


import socket
from . import funcargparse, strpack, general


class SocketError(socket.error):
    """
    Base socket error class.
    """
    def __init__(self, msg):
        socket.error.__init__(self,msg)
        
class SocketTimeout(SocketError):
    """
    Socket timeout error.
    """
    def __init__(self, msg):
        SocketError.__init__(self,msg)


def _wait_sock_func(func, timeout, wait_callback):
    if wait_callback is None:
        return func()
    cnt=general.Countdown(timeout)
    while True:
        try:
            return func()
        except socket.timeout:
            wait_callback()
            if cnt.passed():
                raise

class ClientSocket(object):
    """
    A client socket (used to connect to a server socket).
    
    Args:
        sock (socket.socket): If not ``None``, use already created socket.
        timeout (float): The timeout used for connecting and sending/receving.
        wait_callback (Callable): Called periodically (every 100ms by default) while waiting for connecting or sending/receiving.
        send_method (str): Default sending method.
        recv_method (str): Default receiving method.
    
    Possible sending/receiving methods are:
        - ``'fixedlen'``: data is sent as is, and receiving requires to know the length of the message;
        - ``'decllen'``: data is prepended by a length, and receving reads this length and doesn't need predetermined length info.
        
    Attributes:
        sock (socket.socket): Correpsonding Python socket.
        decllen_bo (str): Byteorder of the prependend length for ``'decllen'`` sending method.
            Can be either ``'>'`` (big-endian, default) or ``'<'``.
        decllen_ll (int): Length of the prependend length for ``'decllen'`` sending method; default is 4 bytes.
    """
    _default_wait_callback_timeout=0.1
    def __init__(self, sock=None, timeout=None, wait_callback=None, send_method="decllen", recv_method="decllen"):
        funcargparse.check_parameter_range(send_method,"send_method",{"fixedlen","decllen"})
        funcargparse.check_parameter_range(recv_method,"recv_method",{"fixedlen","decllen"})
        object.__init__(self)
        self.sock=sock or socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.timeout=timeout
        self.wait_callback=wait_callback
        if wait_callback is not None:
            self.sock.settimeout(self._default_wait_callback_timeout)
        elif timeout is not None:
            self.sock.settimeout(timeout)
        self.send_method=send_method
        self.recv_method=recv_method
        self.decllen_bo=">"
        self.decllen_ll=4
        
    def set_wait_callback(self, wait_callback=None):
        """Set callback function for waiting during connecting or sending/receiving."""
        self.wait_callback=wait_callback
        if wait_callback is not None:
            self.sock.settimeout(self._default_wait_callback_timeout)
        else:
            self.sock.settimeout(self.timeout)
    def set_timeout(self, timeout=None):
        """Set timeout for connecting or sending/receiving."""
        self.timeout=timeout
        if self.wait_callback is None:
            self.sock.settimeout(self.timeout)
    
    def _connect_callback(self):
        self.sock.close()
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.sock.settimeout(self._default_wait_callback_timeout)
        if self.wait_callback:
            self.wait_callback()
    def connect(self, host, port):
        """Connect to a remote host."""
        sock_func=lambda: self.sock.connect((host,port))
        return _wait_sock_func(sock_func,self.timeout,self._connect_callback)
    def close(self):
        """Close the connection."""
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        
    def _recv_wait(self, l):
        sock_func=lambda: self.sock.recv(l)
        return _wait_sock_func(sock_func,self.timeout,self.wait_callback)
    def _send_wait(self, msg):
        sock_func=lambda: self.sock.send(msg)
        return _wait_sock_func(sock_func,self.timeout,self.wait_callback)
                    
    def recv_fixedlen(self, l):
        """Receive fixed-length message of length `l`."""
        buf=""
        while len(buf)<l:
            try:
                recvd=self._recv_wait(l-len(buf))
            except socket.timeout:
                raise SocketTimeout("timeout while receiving")
            if len(recvd)==0:
                raise SocketError("connection closed while receiving")
            buf=buf+recvd
        return buf
    def recv_delimiter(self, delim, lmax=None, chunk_l=1024):
        """
        Receive a single message ending with a delimiter `delim` (can be several characters).
        
        `lmax` specifies the maximal received length (`None` means no limit).
        `chunk_l` specifies the size of data chunk to be read in one try.
        """
        buf=""
        while not buf.endswith(delim):
            try:
                recvd=self._recv_wait(chunk_l)
            except socket.timeout:
                raise SocketTimeout("timeout while receiving")
            if len(recvd)==0:
                raise SocketError("connection closed while receiving")
            buf=buf+recvd
            if len(buf)>lmax:
                break
        return buf
    def recv_decllen(self):
        """
        Receive variable-length message (prepended by its length).
        
        Length format is described by `decllen_bo` and `decllen_ll` attributes.
        """
        len_msg=self.recv_fixedlen(self.decllen_ll)
        l=strpack.unpack_uint(len_msg,self.decllen_bo)
        return self.recv_fixedlen(l)
    def recv(self, l=None):
        """
        Receive a message using the default method.
        """
        if self.send_method=="decllen":
            return self.recv_decllen()
        else:
            return self.recv_fixedlen(l)
    def recv_ack(self, l=None):
        """Receive a message using the default method and send an acknowledgement (message length)."""
        msg=self.recv(l=l)
        ack_msg=strpack.pack_uint(len(msg),self.decllen_ll,self.decllen_bo)
        self.send_fixedlen(ack_msg)
        return msg
    
    def send_fixedlen(self, msg):
        """Send a message as is."""
        sent_total=0
        while sent_total<len(msg):
            try:
                sent=self._send_wait(msg[sent_total:])
            except socket.timeout:
                raise SocketTimeout("timeout while sending")
            if sent==0:
                raise SocketError("connection closed while sending")
            sent_total=sent_total+sent
        return sent_total
    def send_decllen(self, msg):
        """
        Send a message as a variable-length (prepending its length in the sent message).
        
        Length format is described by `decllen_bo` and `decllen_ll` attributes.
        """
        len_msg=strpack.pack_uint(len(msg),self.decllen_ll,self.decllen_bo)
        return self.send_fixedlen(len_msg+msg)-len(len_msg)
    def send_delimiter(self, msg, delimiter):
        """
        Send a message with a delimiter `delim` (can be several characters).
        """
        return self.send_fixedlen(msg+delimiter)-len(delimiter)
    def send(self, msg):
        """
        Send a message using the default method.
        """
        if self.send_method=="decllen":
            return self.send_decllen(msg)
        else:
            return self.send_fixedlen(msg)
    def send_ack(self, msg):
        """
        Send a message using default method and wait for acknowledgement (message length).
        
        If the acknowledgement message length doesn't agree, raise :exc:`SocketError`.
        """
        res=self.send(msg)
        ack_msg=self.recv_fixedlen(self.decllen_ll)
        l=strpack.unpack_uint(ack_msg,self.decllen_bo)
        if l!=len(msg):
            raise SocketError("acknowledgement message contains wrong length: expect {}, got {}".format(len(msg),l))
        return res
        
        
        
        
        
_listen_wait_callback_timeout=0.1
def listen(host, port, conn_func, wait_callback=None, timeout=None, backlog=10, wrap_socket=True):
    """
    Run a server socket at the given host and port.
    
    Args:
        host (str): Server host address. If ``None``, use the local host defined by :func:`socket.gethostname`.
        port (int): Server port.
        conn_func (Callable): Called with the client socket as a single argument every time a connection is established.
        wait_callback (Callable): A callback function which is called periodically (every 100ms by default) while awaiting for connections.
        timeout (float): Timeout for waiting for the connections (``None`` is no timeout).
        backlog (int): Backlog length for the socket (see :func:`socket.socket.listen`).
        wrap_socket (bool): If ``True``, wrap the clinet socket of the connection into :class:`ClientSocket` class;
            otherise, return :class:`socket.socket` object.
        
    Checking for connections is paused unitl `conn_func` returns.
    If multiple connections are expected, `conn_func` should spawn a separate processing thread and return.
    """
    if host is None:
        host=socket.gethostbyname(socket.gethostname())
    serv_sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    if wait_callback is not None:
        serv_sock.settimeout(_listen_wait_callback_timeout)
    elif timeout is not None:
        serv_sock.settimeout(timeout)
    serv_sock.bind((host,port))
    serv_sock.listen(backlog)
    def sock_func():
        client_sock,_=serv_sock.accept()
        if wrap_socket:
            client_sock=ClientSocket(client_sock)
        conn_func(client_sock)
    try:
        while True:
            _wait_sock_func(sock_func,timeout,wait_callback)
    finally:
        serv_sock.close()