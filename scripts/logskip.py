# utility to throttle logging
import rospy,logging,time
EVERY=30
INTERVAL=2
def _logskip(lggr,*args):
	_logskip.counter+=1;
	now=time.time()
	if _logskip.counter%EVERY!=0 or now-_logskip.last_time<INTERVAL:
		return
	_logskip.last_time=now
	lggr(*args)
	
_logskip.counter=-1
_logskip.last_time=time.time()

def info(*args):
	_logskip(logging.getLogger('rosout').info, *args)
def debug(*args):
	_logskip(logging.getLogger('rosout').debug, *args)
def warn(*args):
	_logskip(logging.getLogger('rosout').warning, *args)
