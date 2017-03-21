""" file_db

This class is a very simple interface to a file-based key-value store (i.e., a database).
It enforces file locking, so the caller shouldn't have to worry about concurrent reads/writes.


Eric Schneider <eric.schneider@kcl.ac.uk>

"""

from lockfile import LockFile, LockTimeout, NotLocked
import os.path
import shelve

import rospkg
import rospy


class FileDB:

    def __init__(self, filename=None):
        # The database object
        self.db = None

        # A file-based lock to prevent concurrent reads/writes. This needs to be
        # file-based because it will sometime be run on a cluster, where jobs
        # (and access to this database) may run in parallel with no inter-node
        # communication.
        self.lock = None

        # If the caller passed a filename, open it now
        if filename:
            self.open(filename)

    @staticmethod
    def _get_base_path():
        pkg = rospkg.RosPack()
        return pkg.get_path('mrta')

    def open(self, filename):
        filepath = os.path.join(self._get_base_path(), filename)

        print("Opening file database {0}".format(filepath))

        self.lock = LockFile(filepath)
        while not self.lock.i_am_locking():
            try:
                self.lock.acquire(timeout=10)  # wait up to 60 seconds
            except LockTimeout:
                self.lock.break_lock()
                self.lock.acquire()

        # Any exception here (e.g., DBAccessError) should be handled by the caller
        self.db = shelve.open(filepath)

    def close(self):
        if self.db is not None:
            self.db.sync()
            self.db.close()
            self.db = None

        if self.lock is not None:
            try:
                self.lock.release()
            except NotLocked:
                pass
            self.lock = None

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def __del__(self):
        self.close()

    def __getitem__(self, key):
        """ Overload the [] operator for getting values """

        # Complain if we haven't opened the database yet
        if self.db is None:
            raise IOError

        return self.db[key]

    def __setitem__(self, key, item):
        """ Overload the [] operator for getting values """

        # Complain if we haven't opened the database yet
        if self.db is None:
            raise IOError

        self.db[key] = item

    def exists(self, key):
        return key in self.db
