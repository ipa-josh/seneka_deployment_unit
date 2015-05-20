#!/usr/bin/env python
import rospy
import seneka_pnp.srv
import signal
import sys

shutdown_req=False
def signal_handler(signal, frame):
        global shutdown_req
        print('You pressed Ctrl+C!')
        shutdown_req=True
        

print "init",
rospy.init_node('seneka_demo')
        
signal.signal(signal.SIGINT, signal_handler)

rospy.wait_for_service('/seneka_pnp/getState')
rospy.wait_for_service('/seneka_pnp/setTransition')
print "..."

srv_getState = rospy.ServiceProxy('/seneka_pnp/getState', seneka_pnp.srv.getState)
srv_setTransisition = rospy.ServiceProxy('/seneka_pnp/setTransition', seneka_pnp.srv.setTransition)
print "done"


def getState():
	try:
	  resp = srv_getState()
	  return resp.state
	except rospy.ServiceException as exc:
	  print("Service did not process request: " + str(exc))
	return "error"
	
def setTransisition(tr):
	print "-> "+tr
	try:
	  resp = srv_setTransisition(tr)
	  return resp.transition==tr
	except rospy.ServiceException as exc:
	  print("Service did not process request: " + str(exc))
	return False

def setTransisitionAndWait(tr, exp, cur):
	setTransisition(tr)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		st = getState()
		if st==cur:
			rate.sleep()
		elif st==exp:
			return True
		else:
			print "could not reach expected state ("+exp+") != "+st
			return False
	return False
			
st=""
running=True
while running:
	if (st=="collision_free" or st=="home" or st=="packed-rear") and (rospy.is_shutdown() or shutdown_req):
		break
	st = getState()
	
	#at startup
	if st=="collision_free":
		setTransisitionAndWait("toHome", "home", st)
	elif st=="home":
		setTransisitionAndWait("homeToPreGraspRear", "pregrasp-rear", st)
		
	#loop
	elif st=="pregrasp-rear" or st=="deployed-rear":
		if st=="deployed-rear":
			setTransisition("payloadWithNode")
		setTransisitionAndWait("toPrePackRear", "prepack-rear", st)
		if st=="pregrasp-rear":
			setTransisition("payloadWithNode")
			print "insert sensor node"
			raw_input("Press Enter to continue...")
	elif st=="prepack-rear":
		setTransisitionAndWait("toPackedRear", "packed-rear", st)
	elif st=="packed-rear":
		setTransisitionAndWait("deployRear", "deploy-rear", st)
	elif st=="deploy-rear":
		setTransisitionAndWait("toPrePackRear", "prepack-rear", st)
		#setTransisitionAndWait("deployRearDrop", "deployed-rear", st)
		
	#failure case
	else:
		print "unknown state: "+st
		print "please go back to a safe starting position"
		print "exiting..."
		exit()
		
if st==st=="packed-rear":
	print "remove sensor node"
	setTransisition("payloadWithoutNode")
	raw_input("Press Enter to continue to get back to home position...")
	setTransisition("packedRearDropToHome")

print "last state was: ", st
