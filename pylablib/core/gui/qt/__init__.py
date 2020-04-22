try:
    from PyQt5 import QtGui, QtWidgets, QtCore
    from PyQt5.QtCore import pyqtSignal as Signal, pyqtSlot as Slot
    from sip import delete as qdelete
except ImportError:
    from PySide2 import QtGui, QtWidgets, QtCore
    from PySide2.QtCore import Signal, Slot
    from shiboken2 import delete as qdelete
    
from .thread import *
from .widgets import *