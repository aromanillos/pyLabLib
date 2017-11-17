from builtins import zip
from ..utils.py3 import textstring

from ..utils import iterator as iterator_utils #@UnresolvedImport
from ..utils import funcargparse, general, strdump
from . import table_storage, column, indexing

import numpy as np

_depends_local=[".table_storage","..utils.strdump"]


_storage_types={"columns":table_storage.ColumnDataTableStorage,"array":table_storage.ArrayDataTableStorage}
_default_storage_type=table_storage.ColumnDataTableStorage

class DataTable(object):
    """
    A data table which is designed to store data in several named heterogeneous columns.

    Differences from the regular numpy array:
        - The columns have names which can be used for indexing; this proves to be more convenient for large (10+ columns) datatables
        - Different columns can have different types, so, e.g., non-numeric datatypes (such as str) still allow to preserve the rest of the data as numeric
        - Automatic typecasting for assignment (e.g., if the table starts as `int`, part of it can be assigned `float` without re-creating the whole table)
        - Various useful methods: insert, append, modified.

    Args:
        data: table data; can be a numpy array, a list of columns, a 2D list, or a dict of data columns (in which case `column_names` determines which columns to get from the dict)
        column_names(list): list of column names; by default, the column names are autogenerated: ``"col00"``, ``"col01"``, etc.
        transposed: if ``True``, the `columns` arguments is assumed to be column-wise (list of columns)
            if ``False``, the `columns` arguments is assumed to be row-wise (list of rows)
            if ``"auto"``, assumed to be ``False`` for numpy arrays and ``True`` otherwise
        force_copy (bool): if ``True``, make sure that the supplied data is copied
        storage_type (str): determines the type of underlying DataTable storage:
            ``'columns'`` (default) stores each column separately in an :class:`IDataColumn` object;
            ``'array'`` stores all the data in a 2D numpy array (limited functionality, but faster execution)
    """
    def __init__(self, data=None, column_names=None, transposed="auto", force_copy=True, storage_type=None):
        object.__init__(self)
        if isinstance(data, table_storage.IDataTableStorage):
            if force_copy:
                data=data.copy()
            self._storage=data
        elif isinstance(data, DataTable):
            if force_copy:
                self._storage=data._storage.copy()
            else:
                self._storage=data._storage
        else:
            if storage_type is None:
                storage_type=_default_storage_type
            else:
                storage_type=_storage_types[storage_type]
            if isinstance(data,dict):
                if column_names is not None:
                    data=[data[c] for c in column_names]
                else:
                    data,column_names=data.values(),data.keys()
            self._storage=storage_type(data,column_names,transposed=transposed,force_copy=force_copy)
        self._set_accessors()
        self._x_col=None
        
    def _set_accessors(self):
        self.c =self.ColumnAccessor(self._storage,False)
        self.ca=self.ColumnAccessor(self._storage,True )
        self.r =self.RowAccessor(self._storage,False)
        self.ra=self.RowAccessor(self._storage,True )
        self.t =self.TableAccessor(self._storage,False)
        self.ta=self.TableAccessor(self._storage,True )
    
    ## Shape ##
    @property   # property for compatibility with np.shape
    def shape(self):
        return self._storage.shape
    @property   # property for compatibility with np.ndim
    def ndim(self):
        return 2
    def nrows(self):
        """Get number of rows."""
        return self.shape[0]
    def ncols(self):
        """Get number of columns."""
        return self.shape[1]
    def __len__(self):
        return self.nrows()
    def idx(self):
        """Create an index column (analogous to ``np.arange(len(table))``)."""
        return column.LinearDataColumn(self.nrows())
    
    ## Iterators ##
    def __iter__(self):
        return self.ra.__iter__()
    
    ## Casting to NumPy array ##
    def as_array(self, force_copy=False):
        """
        Turn the table into a numpy array.

        If ``force_copy==True``, ensure that the result is a copy of the data.
        """
        return self._storage.as_array(force_copy=force_copy)
    __array__=as_array # property for compatibility with np.ufuncs
    
    ## Accessors ##
    class RowAccessor(object):
        """
        A row accessor: creates a simple interface to treat the table row-wise (append/insert/delete/iterate over rows).

        Generated automatically for each table on creation, doesn't need to be created explicitly.
        """
        def __init__(self, storage, return_numpy=True):
            object.__init__(self)
            self._storage=storage
            self.return_numpy=return_numpy
        def __iter__(self):
            if self.return_numpy:
                return iterator_utils.AccessIterator(self, lambda obj, idx: obj._storage.get_single_row_item(idx))
            else:
                return iterator_utils.AccessIterator(self, lambda obj, idx: obj._storage.get_single_row(idx))
        def __getitem__(self, idx):
            if self.return_numpy:
                return self._storage.get_item((idx,slice(None)))
            else:
                return self._storage.get_rows(idx)
        def __setitem__(self, idx, val):
            self._storage.set_rows(idx, val)
        def __delitem__(self, idx):
            self._storage.del_rows(idx)
        def insert(self, idx, val):
            """Insert a row or a list of rows to index `idx` (1D)."""
            if idx is None:
                idx=self._storage.shape[0]
            self._storage.add_rows(idx,val)
        def append(self, val):
            """Append a row or a list of rows to the end of the table."""
            self.insert(None,val)
        def idx(self):
            """Create an index column (analogous to ``np.arange(len(table))``)."""
            return self._storage.idx()
    
    class ColumnAccessor(object):
        """
        A column accessor: creates a simple interface to treat the table column-wise (append/insert/delete/iterate over columns).

        Generated automatically for each table on creation, doesn't need to be created explicitly.
        """
        def __init__(self, storage, return_numpy=True):
            object.__init__(self)
            self._storage=storage
            self.return_numpy=return_numpy
        def __iter__(self):
            return iterator_utils.AccessIterator(self)
        def keys(self):
            """Get keys (column names)"""
            return self._storage.get_column_names()
        def iterkeys(self):
            """Iterate over keys (column names)"""
            return iterator_utils.AccessIterator(self, lambda obj, idx: obj._storage.get_column_names()[idx])
        def itervalues(self):
            """Iterate over values (`IDataColumn` objects)"""
            return iterator_utils.AccessIterator(self, lambda obj, idx: obj._storage.get_single_column(idx))
        def iteritems(self):
            """Iterate over items (tuples of column names and corresponding `IDataColumn` objects)"""
            if self.return_numpy:
                return iterator_utils.AccessIterator(self, lambda obj, idx: (obj._storage.get_column_names()[idx],np.asarray(obj._storage.get_single_column(idx))) )
            else:
                return iterator_utils.AccessIterator(self, lambda obj, idx: (obj._storage.get_column_names()[idx],obj._storage.get_single_column(idx)) )
        def __getitem__(self, idx):
            if self.return_numpy:
                return self._storage.get_item((slice(None),idx))
            else:
                return self._storage.get_columns(idx)
        def __setitem__(self, idx, val):
            self._storage.set_columns(idx, val)
        def __delitem__(self, idx):
            self._storage.del_columns(idx)
        def insert(self, idx, val, names=None, transposed="auto"):
            """
            Add new columns at index `idx` (1D).
        
            Columns data is given by `val` and their names are given by `names` (a string for a single column, or a list of strings for multiple columns).
            If ``transposed==True``, `val` is assumed to be arranged column-wise (list of columns).
            If ``transposed==False``, `val` is assumed to be arranged row-wise (list of rows).
            If ``transposed=="auto"``, it is assumed to be ``True`` if `val` is a 2D numpy array, and ``False`` otherwise.
            """
            if idx is None:
                idx=self._storage.shape[1]
            self._storage.add_columns(idx,val,names,transposed=transposed)
        def append(self, val, names=None, transposed="auto"):
            """
            Append a column or a list of column to the end of the table.
            
            See :func:`DataTable.add_columns` for description.
            """
            self.insert(None,val,names,transposed=transposed)
        def move(self, source, dest):
            """
            Move a column with the `source` index to the `dest` position (should be integer).

            The column names are adjusted accordingly.
            """
            source_name=self._storage.get_column_names(source)
            source_column=self[source]
            self._storage.del_columns(source) 
            self._storage.add_columns(dest,source_column,source_name,force_copy=False)
        def idx(self):
            """Create an index row (analogous to ``np.arange(table.shape[1])``)."""
            return column.LinearDataColumn(self._storage.ncols())
    
    class TableAccessor(object):
        """
        A table accessor: acessing the table data through this interface returns DataTable objects (acessing DataTable object directly returns numpy objects for better compatibility).

        Generated automatically for each table on creation, doesn't need to be created explicitly.
        """
        def __init__(self, storage, return_numpy=True):
            object.__init__(self)
            self._storage=storage
            self.return_numpy=return_numpy
        def __iter__(self):
            return iterator_utils.AccessIterator(self)
        def __getitem__(self, idx):
            if self.return_numpy:
                return self._storage.get_item((slice(None),idx))
            else:
                return DataTable(self._storage.get_subtable(idx),force_copy=False)
        def __setitem__(self, idx, val):
            self._storage.set_item(idx, val)
        def idx(self):
            return self._storage.idx()
    def expand(self, length):
        """
        Expand the table by `length`. Usually fill with zeros, unless the column values can be auto-predicted.
        """
        self._storage.expand(length)
    def resize(self, length):
        """
        Resize the table to `length`.

        If the curent length is larger, the table is truncated. Otherwise, it's expanded (using :func:`expand`). 
        """
        diff=length-self.nrows()
        if diff<0:
            del self.r[length:]
        elif diff>0:
            self.expand(diff)
        
    ## Default indexing ##
    def __getitem__(self, idx):
        return self._storage.get_item(idx)
    def __setitem__(self, idx, val):
        self._storage.set_item(idx,val)
        
    ## Columns indexing ##
    def get_column_names(self, idx=None):
        """Return the list of column names at index `idx` (by default, all of the names)."""
        return self._storage.get_column_names(idx)
    def set_column_names(self, new_names):
        """Return the list of column names."""
        self._storage.set_column_names(new_names)
    def change_column_names(self, idx, val):
        """
        Change names of columns at index `idx`.
        """
        if idx is None:
            idx=slice(None)
        c_ndim,idx=indexing.to_list_idx_noslice(idx,self.get_column_names()).tup()
        new_names=list(self.get_column_names())
        if c_ndim==0:
            new_names[idx]=val
        else:
            if isinstance(val,textstring):
                raise ValueError("can't assign single name to multiple columns")
            for i,v in zip(idx,val):
                new_names[i]=v
        self.set_column_names(new_names)
    def swap_columns(self, idx1, idx2):
        """
        Swap two columns at indicex `idx1` and `idx2`.

        Names are adjusted accordingly.
        """
        self._storage.swap_columns(idx1,idx2)
    def rearrange_columns(self, idx):
        """
        Rearrange columns according to the new index (analogous to replacing the table by its subtable given by index `idx`).
        """
        self._storage=self._storage.get_subtable((slice(None),idx))
        self._set_accessors() 
    def modified(self, idx, val, names=None, force_copy=True):
        """
        Return a copy of this table with the columns at index `idx` modified.

        `val` can be either a single number or column (if `idx` is a single index), or a list of numbers or columns (if `idx` is a list of indices).
        `names` specifices new names of modified columns (by default the old names are preserved).
        If ``force_copy==True``, all of the table data is deep-copied.
        """
        if idx is None:
            idx=slice(None)
        c_ndim,idx=indexing.to_list_idx_noslice(idx,self.get_column_names()).tup()
        columns=list(self.c.iteritems())
        v_ndim=len(table_storage.get_shape(val))
        if c_ndim==0:
            n=columns[idx][0] if names is None else names
            if v_ndim==0:
                val=[val]*len(self._storage.get_single_column(idx))
            columns[idx]=(n,val)
        else:
            if v_ndim==0 or len(idx)!=len(val):
                raise ValueError("numbers of columns don't coincide for idx and val")
            if names is None:
                names=[None]*len(idx)
            elif len(idx)!=len(names):
                raise ValueError("numbers of columns don't coincide for idx and names")
            for i,v,n in zip(idx,val,names):
                if len(table_storage.get_shape(v))==0:
                    v=[v]*len(self._storage.get_single_column(i))
                n=columns[i][0] if n is None else n
                columns[i]=(n,v)
        names,values=zip(*columns)
        return DataTable(values,names,transposed=True,force_copy=force_copy)
        
    ## Special column types ##
    def set_x_column_name(self, idx="#"):
        """
        Set the dedicated x column.

        Can be either a valid column name (or numeric index), or ``"#"`` which means the index column (returned by :func:`idx`).
        """
        if idx!="#":
            self.c[idx] # check validity
        self._x_col=idx
    def get_x_column_name(self):
        """
        Get the name of the dedicated.
        """
        if self._x_col is None:
            return 0 if self.ncols()>0 else "#"
        return self._x_col
    def get_x_column(self, x_column=None, idx_default=False):
        """
        Get the dedicated x column.
        """
        if np.ndim(x_column)>0:
            return x_column
        if x_column is None:
            x_column=self.get_x_column_name()
        if x_column=="#":
            return self.idx()
        return self.c[x_column]
    
    ## Copying ##
    def copy(self, copy_columns=True):
        """
        Copy the table.

        If ``copt_columns==True``, deep-copy the columns as well.
        """
        if copy_columns:
            return DataTable(self._storage,force_copy=True)
        else:
            return DataTable(self.c[:],self.get_column_names(),transposed=True,force_copy=False)
        
    ## Extending ##
    def concatenate(self, table, diff_columns_action="exclude", fill_value=np.nan):
        if not isinstance(table,DataTable):
            raise ValueError("con only concatenate with another DataTable")
        funcargparse.check_parameter_range(diff_columns_action,"diff_columns_action",{"exclude","fill"})
        c1,c2=self.get_column_names(),table.get_column_names()
        cc=set(c1) & set(c2)
        if diff_columns_action=="exclude":
            cc=general.sort_set_by_list(cc,c1)
            t1=self.t[:,cc]
            t2=table.t[:,cc]
            t1.r.append(t2)
        else:
            c1e=general.sort_set_by_list(set(c1)-cc,c1)
            c2e=general.sort_set_by_list(set(c2)-cc,c2)
            t1=self.copy()
            t1.c.append(fill_value,c2e)
            t2=table.copy()
            t2.c.append(fill_value,c1e)
            t2.rearrange_columns(t1.get_column_names())
            t1.r.append(t2)
        return t1
    
    ## Repr ##
    def __str__(self):
        return self._storage.__str__()
    def __repr__(self):
        s=str(self).replace("\n ","\n"+" "*6)
        return "{0}(columns={1},\ndata={2})".format(type(self).__name__,str(self.get_column_names()),s)
    
    ## External functions adding ##
    @classmethod
    def add_array_function(cls, func, alias=None):
        """
        Add function to the class definition, which is automatically applied to the array representation.
        """
        def self_func(self, *args, **vargs):
            return func(self.as_array(force_copy=False),*args,**vargs)
        if alias is None:
            alias=func.__name__
        try:
            self_func.__doc__=func.__doc__
        except AttributeError:
            pass
        setattr(cls,alias,self_func)
    @classmethod
    def add_columnwise_function(cls, func, alias=None, collection_type="list", column_arg_name=None):
        """
        Add function to the class definition, which is automatically applied to each column.
        Results are collected in containers depending on collection_type.
        """
        if not collection_type in ["list","array","table"]:
            raise ValueError("unrecognized collection type: {}".format(collection_type))
        def self_func(self, *args, **vargs):
            c=vargs.pop(column_arg_name,None)
            if c is None:
                res=[func(sc,*args,**vargs) for sc in self.c]
                if collection_type=="array":
                    res=np.array(res)
                elif collection_type=="table":
                    res=DataTable(res,self.get_column_names())
                return res
            else:
                return func(self[c],*args,**vargs)
        if alias is None:
            alias=func.__name__
        try:
            self_func.__doc__=func.__doc__
        except AttributeError:
            pass
        setattr(cls,alias,self_func)
    @classmethod
    def add_column_function(cls, func, alias=None, column_arg_name="column", column_arg_default=None):
        """
        Add function to the class definition, which is automatically applied to a single column.
        Column number should be given as a first argument of the function.
        """
        def self_func(self, *args, **vargs):
            if len(args)>0:
                c=args[0]
                args=args[1:]
            else:
                c=vargs.pop(column_arg_name,column_arg_default)
            if c is None:
                raise TypeError("argument '{}' is not supplied".format(column_arg_name))
            return func(self.c[c],*args,**vargs)
        if alias is None:
            alias=func.__name__
        try:
            self_func.__doc__=func.__doc__
        except AttributeError:
            pass
        setattr(cls,alias,self_func)
        
DataTable.add_column_function(np.argsort)
DataTable.add_column_function(np.nonzero)
DataTable.add_column_function(np.unique)
DataTable.add_columnwise_function(np.argmin, column_arg_name="column")
DataTable.add_columnwise_function(np.argmax, column_arg_name="column")
DataTable.add_columnwise_function(np.min,"min", column_arg_name="column")
DataTable.add_columnwise_function(np.max,"max", column_arg_name="column")
DataTable.add_columnwise_function(np.mean, column_arg_name="column")
DataTable.add_columnwise_function(np.std, column_arg_name="column")
DataTable.add_columnwise_function(np.sum, column_arg_name="column")


### strdump definitions ###
def _dump_datatable(data, dumpf):
    names=data.get_column_names()
    cols=[dumpf(c) for c in data.c]
    return names,cols
def _load_datatable(data, loadf):
    names,cols=data
    columns=[loadf(c) for c in cols]
    return DataTable(columns,names,transposed=True,force_copy=False)
strdump.dumper.add_class(DataTable,_dump_datatable,_load_datatable,"datatable",recursive=True)