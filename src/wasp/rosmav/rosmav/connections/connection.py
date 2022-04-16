from abc import ABCMeta, abstractmethod
import logging

logging.basicConfig(format="[%(levelname)s] %(asctime)s %(message)s", level=logging.DEBUG)
logger = logging.getLogger(__name__)

class Connection(object):
    def __init__(self):
        self._message_listeners = {}

    def add_message_listener(self, name, fn):
        """
        Adds a callback function for the specified message to the set of
        listeners to be triggered when messages arrive.

        Args:
            name: `MsgID` name of the message
            fn: function to trigger when the message is received
        """

        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)

    def notify_message_listeners(self, name, msg):
        """
        Triggers all callbacks for a given message.

        Goes through list of registered listeners (callback functions) for the
        MsgID and calls each function sequentially.

        Args:
            name: `MsgID` name of the message
            msg: message data to pass to each of the listeners
        """
        for fn in self._message_listeners.get(name, []):
            try:
                fn(name, msg)
            except Exception as e:
                logger.error("notify error", exc_info=True)

        