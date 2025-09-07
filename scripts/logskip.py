# utility to throttle logging
import rospy,logging,time
EVERY=30 # skip this many between each logging
INTERVAL=2 # minimum interval in seconds between log messages

def _logskip(lggr,*args):
	_logskip.counter+=1;
	now=time.time()
	if _logskip.counter%EVERY!=0 or now-_logskip.last_time<INTERVAL:
		return
	_logskip.last_time=now
	lggr(*args)
	
_logskip.counter=-1
_logskip.last_time=time.time()
_logskip.info=logging.getLogger('rosout').info
_logskip.debug=logging.getLogger('rosout').debug
_logskip.warn=logging.getLogger('rosout').warning

def info(*args):
	_logskip(_logskip.info, *args)
def debug(*args):
	_logskip(_logskip.debug, *args)
def warn(*args):
	_logskip(_logskip.warn, *args)
