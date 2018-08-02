from builtins import bytes as new_bytes

if str is bytes: # Python 2

    textstring=(basestring,)
    bytestring=(str,new_bytes)
    anystring=(str, unicode)

    def as_str(data):
        return data
    def as_bytes(data):
        return new_bytes(data)
    def as_builtin_bytes(data):
        return data

else:

    textstring=(str,)
    bytestring=(bytes,)
    anystring=(str,bytes)

    def as_str(data):
        return data if isinstance(data,str) else data.decode()
    def as_bytes(data):
        return data if isinstance(data,bytes) else (data.encode("utf-8") if isinstance(data,str) else bytes(data))
    as_builtin_bytes=as_bytes

def as_datatype(data, datatype):
    if datatype=="auto":
        return data
    elif datatype=="str":
        return as_str(data)
    return as_bytes(data)