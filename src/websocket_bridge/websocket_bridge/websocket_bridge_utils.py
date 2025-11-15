
from threading import Lock

from std_msgs.msg import Header

from typing import Any


class AtomicBool:
    """
    Class to represent a boolean value that can be safely accessed and modified.
    """

    def __init__(self, initial: bool = False) -> None:
        """
        Constructor.

        :param initial: Initial value of the boolean, defaults to False.
        """
        self._value = initial
        self._lock = Lock()

    def store(self, new_value: bool) -> None:
        """
        Sets the boolean value in a thread-safe manner.

        :param new_value: The new boolean value to be stored.
        """
        with self._lock:
            self._value = new_value

    def load(self) -> bool:
        """
        Retrieves the current boolean value in a thread-safe manner.

        :return: The current boolean value.
        """
        with self._lock:
            return self._value
